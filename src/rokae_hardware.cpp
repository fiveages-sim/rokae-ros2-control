#include "rokae_ros2_control/rokae_hardware.h"
#include "rokae_ros2_control/ag2f90d_gripper.h"
#include "rokae_ros2_control/jodell_gripper.h"
#include "rokae_ros2_control/erg32_gripper.h"
#include "rokae_ros2_control/linkerhand.h"
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
#include <iomanip>
#include <type_traits>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>

namespace rokae_ros2_control {
    namespace {
        constexpr double kRealtimeControlDt = 0.001;
        constexpr double kEpsilon = 1.0e-9;
        constexpr bool kSdkFilterLimitEnable = true;
        constexpr int kSdkFilterLimit = 10;
        constexpr int kSdkFilterJointFrequency = 10;
        constexpr int kSdkFilterCartesianFrequency = 10;
        constexpr int kSdkFilterTorqueFrequency = 10;

        std::string normalizeControlMode(const std::string &mode) {
            std::string normalized = mode;
            std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                           [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
            normalized.erase(
                std::remove_if(normalized.begin(), normalized.end(),
                               [](unsigned char c) { return std::isspace(c) || c == '-'; }),
                normalized.end());

            if (normalized == "JOINTIMPEDANCE") {
                return "JOINT_IMPEDANCE";
            }
            return normalized;
        }

        std::vector<double> parseDoubleArrayLoose(
            const std::string &str,
            const std::vector<double> &default_value) {
            if (str.empty()) {
                return default_value;
            }

            std::string cleaned = str;
            cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '['), cleaned.end());
            cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), ']'), cleaned.end());
            std::replace(cleaned.begin(), cleaned.end(), ',', ' ');

            std::istringstream iss(cleaned);
            std::vector<double> values;
            double value = 0.0;
            while (iss >> value) {
                values.push_back(value);
            }
            return values.empty() ? default_value : values;
        }

        std::string detectGripperJoint(const std::vector<hardware_interface::ComponentInfo>& joints) {
            for (const auto& joint : joints) {
                std::string joint_name_lower = joint.name;
                std::transform(joint_name_lower.begin(),
                               joint_name_lower.end(),
                               joint_name_lower.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

                if (joint_name_lower.find("gripper") != std::string::npos ||
                    joint_name_lower.find("hand") != std::string::npos) {
                    return joint.name;
                }
            }

            return "";
        }

        bool isAg2f90CType(const std::string &gripper_type) {
            return gripper_type == "ag2f90_c" || gripper_type == "ag2f90c" ||
                   gripper_type == "ag2f90" || gripper_type == "changingtek90c";
        }

        bool isAg2f90DType(const std::string &gripper_type) {
            return gripper_type == "ag2f90_d" || gripper_type == "ag2f90d" ||
                   gripper_type == "changingtek90d";
        }

        bool isAg2f90Type(const std::string &gripper_type) {
            return isAg2f90CType(gripper_type) || isAg2f90DType(gripper_type);
        }

        bool gripperJointIsRight(const std::string &joint_name) {
            std::string lower = joint_name;
            std::transform(lower.begin(), lower.end(), lower.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            return lower.find("right") != std::string::npos;
        }

        double clampToolParam(double value) {
            return std::clamp(value, 0.0, 1.0);
        }
    } // namespace

    int RokaeHardware::DoubleSFilter::sign(double value) {
        if (value > kEpsilon) {
            return 1;
        }
        if (value < -kEpsilon) {
            return -1;
        }
        return 0;
    }

    void RokaeHardware::DoubleSFilter::reset(
        double position, double velocity, double acceleration) {
        q = position;
        qd = velocity;
        qdd = acceleration;
    }

    double RokaeHardware::DoubleSFilter::updateTo(
        double target_position, double target_velocity, double target_acceleration) {
        const double jerk = getJerk(target_position, target_velocity, target_acceleration);
        const double next_acceleration = qdd + dt * jerk;
        const double next_velocity = qd + dt * 0.5 * (next_acceleration + qdd);
        q += dt * 0.5 * (next_velocity + qd);
        qd = next_velocity;
        qdd = next_acceleration;
        return q;
    }

    double RokaeHardware::DoubleSFilter::getJerk(
        double target_position, double target_velocity, double target_acceleration) const {
        const double jerk = std::max(std::abs(jerk_limit), kEpsilon);
        const double ek = (q - target_position) / jerk;
        const double ek_1 = (qd - target_velocity) / jerk;
        const double ek_2 = (qdd - target_acceleration) / jerk;
        const double e_min_1 = (v_min - target_velocity) / jerk;
        const double e_max_1 = (v_max - target_velocity) / jerk;
        const double e_min_2 = (a_min - target_acceleration) / jerk;
        const double e_max_2 = (a_max - target_acceleration) / jerk;

        const double delta = ek_1 + (ek_2 * std::abs(ek_2)) / 2.0;
        const int s_delta = sign(delta);
        const double sqrt_arg = std::max(
            0.0,
            2.0 * std::pow((ek_2 * ek_2) + 2.0 * ek_1 * static_cast<double>(s_delta), 3.0));
        const double sigma =
            ek + ek_1 * ek_2 * static_cast<double>(s_delta) -
            (std::pow(ek_2, 3.0) * (1.0 - 3.0 * std::abs(s_delta))) / 6.0 +
            static_cast<double>(s_delta) * std::sqrt(sqrt_arg) / 4.0;

        const double max_accel_error = std::abs(e_max_2) > kEpsilon ? e_max_2 : kEpsilon;
        const double min_accel_error = std::abs(e_min_2) > kEpsilon ? e_min_2 : -kEpsilon;
        const double positive_brake = (ek_2 * ek_2) - 2.0 * ek_1;
        const double negative_brake = (ek_2 * ek_2) + 2.0 * ek_1;
        const double nu_p =
            ek - max_accel_error * positive_brake / 4.0 -
            (positive_brake * positive_brake) / (8.0 * max_accel_error) -
            ek_2 * (3.0 * ek_1 - (ek_2 * ek_2)) / 3.0;
        const double nu_n =
            ek - min_accel_error * negative_brake / 4.0 -
            (negative_brake * negative_brake) / (8.0 * min_accel_error) +
            ek_2 * (3.0 * ek_1 + (ek_2 * ek_2)) / 3.0;

        double sigma_limited = sigma;
        if (ek_2 <= e_max_2 && ek_1 <= (ek_2 * ek_2) / 2.0 - (e_max_2 * e_max_2)) {
            sigma_limited = nu_p;
        } else if (ek_2 >= e_min_2 && ek_1 >= (e_min_2 * e_min_2) - (ek_2 * ek_2) / 2.0) {
            sigma_limited = nu_n;
        }

        const int sigma_sign = sign(sigma_limited);
        const double control =
            -jerk * sign(
                sigma_limited +
                (1.0 - std::abs(sigma_sign)) *
                    (delta + (1.0 - std::abs(s_delta)) * ek_2));

        return std::max(
            velocityControl(e_min_2, e_max_2, ek_2, ek_1, e_min_1),
            std::min(control, velocityControl(e_min_2, e_max_2, ek_2, ek_1, e_max_1)));
    }

    double RokaeHardware::DoubleSFilter::accelControl(
        double error_acceleration, double acceleration_limit) const {
        return -std::abs(jerk_limit) * sign(error_acceleration - acceleration_limit);
    }

    double RokaeHardware::DoubleSFilter::velocityBoundary(
        double error_acceleration, double error_velocity, double velocity_limit) const {
        const double delta_v =
            error_acceleration * std::abs(error_acceleration) +
            2.0 * (error_velocity - velocity_limit);
        return -std::abs(jerk_limit) *
               sign(delta_v + (1.0 - std::abs(sign(delta_v))) * error_acceleration);
    }

    double RokaeHardware::DoubleSFilter::velocityControl(
        double min_error_acceleration,
        double max_error_acceleration,
        double error_acceleration,
        double error_velocity,
        double velocity_limit) const {
        return std::max(
            accelControl(error_acceleration, min_error_acceleration),
            std::min(
                velocityBoundary(error_acceleration, error_velocity, velocity_limit),
                accelControl(error_acceleration, max_error_acceleration)));
    }

    namespace {
        std::mutex startup_group_mutex;
        std::vector<RokaeHardware*> startup_members;
    }

    void RokaeHardware::registerStartupGroup() {
        std::lock_guard<std::mutex> lock(startup_group_mutex);
        if (std::find(startup_members.begin(), startup_members.end(), this) == startup_members.end()) {
            startup_members.push_back(this);
        }
    }

    void RokaeHardware::unregisterStartupGroup() {
        std::lock_guard<std::mutex> lock(startup_group_mutex);
        startup_members.erase(std::remove(startup_members.begin(), startup_members.end(), this), startup_members.end());
    }

    void RokaeHardware::rollbackStartupGroup() {
        // Hold membership stable while stopping siblings; destruction unregisters first.
        std::lock_guard<std::mutex> lock(startup_group_mutex);
        for (auto* member : startup_members) {
            if (member->startup_group_ != startup_group_) continue;
            member->hardware_active_ = false;
            member->rt_control_faulted_ = true;
            member->shutdownRobot();
        }
    }

    RokaeHardware::~RokaeHardware() {
        unregisterStartupGroup();
        shutdownRobot();
    }

    hardware_interface::CallbackReturn RokaeHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params) {
        if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 解析 URDF 配置的关节（直接从 params 读取）
        for (const auto &joint: params.hardware_info.joints) {
            joint_names_.push_back(joint.name);
        }

        // 初始化关节数据存储（根据实际配置的关节数量）
        int joint_count = static_cast<int>(joint_names_.size());
        joint_positions_.resize(joint_count, 0.0);
        joint_velocities_.resize(joint_count, 0.0);
        joint_efforts_.resize(joint_count, 0.0);
        joint_position_commands_.resize(joint_count, 0.0);
        target_joint_position_commands_.resize(joint_count, 0.0);

        // 初始化插值相关变量
        prev_joint_position_commands_.resize(joint_count, 0.0);
        next_joint_position_commands_.resize(joint_count, 0.0);
        last_sent_joint_positions_.resize(joint_count, 0.0);
        alpha_increment_ = 1.0;
        interpolation_alpha_ = 0.0; // 初始插值系数为0

        // 读取配置参数的辅助函数（直接从 params 读取）
        // 支持不同类型的参数读取，自动进行类型转换
        const auto get_hardware_parameter = [&params](const std::string &parameter_name, const auto &default_value) {
            if (auto it = params.hardware_info.hardware_parameters.find(parameter_name);
                it != params.hardware_info.hardware_parameters.end()) {
                if constexpr (std::is_same_v<std::decay_t<decltype(default_value)>, std::string>) {
                    return it->second;
                } else if constexpr (std::is_same_v<std::decay_t<decltype(default_value)>, bool>) {
                    return (it->second == "true" || it->second == "1");
                } else if constexpr (std::is_integral_v<std::decay_t<decltype(default_value)> >) {
                    return static_cast<std::decay_t<decltype(default_value)>>(std::stoi(it->second));
                } else if constexpr (std::is_floating_point_v<std::decay_t<decltype(default_value)> >) {
                    return static_cast<std::decay_t<decltype(default_value)>>(std::stod(it->second));
                }
            }
            return default_value;
        };

        // 读取配置参数
        arm_ip_ = get_hardware_parameter("arm_ip", std::string("192.168.1.160"));
        local_ip_ = get_hardware_parameter("local_ip", std::string("192.168.1.233"));
        const int cpu_affinity = get_hardware_parameter("cpu_affinity", -1);
        rt_cpu_affinity_ = get_hardware_parameter("rt_cpu_affinity", cpu_affinity);
        state_cpu_affinity_ = get_hardware_parameter("state_cpu_affinity", cpu_affinity);
        std::string command_filter_mode =
            get_hardware_parameter("command_filter_mode", std::string("double_s"));
        std::transform(command_filter_mode.begin(), command_filter_mode.end(), command_filter_mode.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        use_double_s_filter_ =
            !(command_filter_mode == "none" || command_filter_mode == "off" || command_filter_mode == "linear");
        command_filter_max_velocity_ =
            std::abs(get_hardware_parameter("command_filter_max_velocity", command_filter_max_velocity_));
        command_filter_max_acceleration_ =
            std::abs(get_hardware_parameter("command_filter_max_acceleration", command_filter_max_acceleration_));
        command_filter_max_jerk_ =
            std::abs(get_hardware_parameter("command_filter_max_jerk", command_filter_max_jerk_));

        const std::string gripper_joint_name = detectGripperJoint(params.hardware_info.joints);
        if (!gripper_joint_name.empty()) {
            std::string gripper_type = get_hardware_parameter("gripper_type", std::string("ag2f90_d"));
            std::transform(gripper_type.begin(), gripper_type.end(), gripper_type.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

            RCLCPP_INFO(get_node()->get_logger(),
                        "Detected Rokae gripper joint '%s', gripper_type=%s",
                        gripper_joint_name.c_str(),
                        gripper_type.c_str());

            const bool is_right_gripper = gripperJointIsRight(gripper_joint_name);
            const double tool_torque = clampToolParam(get_hardware_parameter(
                is_right_gripper ? "right_tool_torque" : "left_tool_torque", 1.0));
            const double tool_velocity = clampToolParam(get_hardware_parameter(
                is_right_gripper ? "right_tool_velocity" : "left_tool_velocity", 1.0));

            if (gripper_type == "jodell" || gripper_type == "rg75" || gripper_type == "dual_rg75") {
                gripper_ = std::make_unique<JodellGripper>(
                    get_node()->get_logger(),
                    get_node()->get_clock(),
                    gripper_joint_name,
                    tool_torque,
                    tool_velocity);
            } else if (gripper_type == "erg32") {
                Erg32Gripper::Config config;
                config.slave_id = get_hardware_parameter("gripper_slave_id", 9);
                config.configure_xpanel = get_hardware_parameter("gripper_configure_xpanel", false);
                config.read_interval_ms = get_hardware_parameter("gripper_read_interval_ms", 100);
                config.feedback_timeout_ms = get_hardware_parameter("gripper_feedback_timeout_ms", 1000);
                config.activation_timeout_ms = get_hardware_parameter("gripper_activation_timeout_ms", 10000);
                config.normalized_torque = tool_torque;
                config.normalized_velocity = tool_velocity;
                try {
                    const auto names = erg32::orderedJoints(joint_names_);
                    if (joint_names_.size() != 9 ||
                        std::find(joint_names_.begin(), joint_names_.begin() + 7, names[0]) != joint_names_.begin() + 7 ||
                        std::find(joint_names_.begin(), joint_names_.begin() + 7, names[1]) != joint_names_.begin() + 7) {
                        throw std::invalid_argument("Expected 7 arm joints followed by 2 ERG32 joints");
                    }
                    gripper_ = std::make_unique<Erg32Gripper>(get_node()->get_logger(),
                        std::vector<std::string>{names[0], names[1]}, config);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_node()->get_logger(), "Invalid ERG32 configuration: %s", e.what());
                    return hardware_interface::CallbackReturn::ERROR;
                }
            } else if (linkerhand::isSupported(gripper_type)) {
                LinkerHand::Config config;
                config.hand_type = gripper_type;
                config.right_hand = is_right_gripper;
                config.slave_id = get_hardware_parameter(
                    "hand_slave_id", is_right_gripper ? 0x27 : 0x28);
                config.configure_xpanel = get_hardware_parameter("hand_configure_xpanel", false);
                config.read_interval_ms = get_hardware_parameter("hand_read_interval_ms", 100);
                config.normalized_torque = tool_torque;
                config.normalized_velocity = tool_velocity;
                try {
                    const auto model = linkerhand::modelFor(gripper_type, is_right_gripper);
                    const auto hand_joints = linkerhand::orderedJoints(model, joint_names_);
                    // Export routines keep arm joints first and export hand interfaces separately.
                    if (joint_names_.size() != 7 + hand_joints.size()) {
                        throw std::invalid_argument("Expected 7 arm joints plus " +
                            std::to_string(hand_joints.size()) + " LinkerHand joints");
                    }
                    for (size_t i = 0; i < joint_names_.size(); ++i) {
                        const bool is_hand = std::find(hand_joints.begin(), hand_joints.end(),
                                                       joint_names_[i]) != hand_joints.end();
                        if (is_hand != (i >= 7)) {
                            throw std::invalid_argument("LinkerHand joints must follow the 7 arm joints");
                        }
                    }
                    gripper_ = std::make_unique<LinkerHand>(
                        get_node()->get_logger(), get_node()->get_clock(), hand_joints, config);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_node()->get_logger(), "Invalid %s configuration: %s",
                                 gripper_type.c_str(), e.what());
                    return hardware_interface::CallbackReturn::ERROR;
                }
            } else if (isAg2f90Type(gripper_type)) {
                Ag2f90dGripper::Config config;
                config.slave_id = get_hardware_parameter("gripper_slave_id", 1);
                config.close_pos = get_hardware_parameter("gripper_close_pos", 9000);
                config.max_opening_distance =
                    get_hardware_parameter("gripper_max_opening_distance", 0.09);
                config.configure_xpanel =
                    get_hardware_parameter("gripper_configure_xpanel", false);
                config.read_interval_ms =
                    get_hardware_parameter("gripper_read_interval_ms", 500);
                config.normalized_torque = tool_torque;
                config.normalized_velocity = tool_velocity;
                config.velocity_reg = get_hardware_parameter("gripper_velocity_reg", config.velocity_reg);
                config.torque_reg = get_hardware_parameter("gripper_torque_reg", config.torque_reg);
                config.acceleration_reg = get_hardware_parameter("gripper_acceleration_reg", config.acceleration_reg);
                config.deceleration_reg = get_hardware_parameter("gripper_deceleration_reg", config.deceleration_reg);
                config.default_acceleration =
                    get_hardware_parameter("gripper_default_acceleration", config.default_acceleration);
                config.default_deceleration =
                    get_hardware_parameter("gripper_default_deceleration", config.default_deceleration);
                if (isAg2f90CType(gripper_type)) {
                    config.model_name = "AG2F90C";
                    config.feedback_reg = Ag2f90dGripper::FEEDBACK_REG_C;
                } else {
                    config.model_name = "AG2F90D";
                    config.feedback_reg = Ag2f90dGripper::FEEDBACK_REG_D;
                }
                config.feedback_reg =
                    get_hardware_parameter("gripper_feedback_reg", config.feedback_reg);
                gripper_ = std::make_unique<Ag2f90dGripper>(
                    get_node()->get_logger(),
                    get_node()->get_clock(),
                    gripper_joint_name,
                    config);
            } else {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Unsupported Rokae gripper_type '%s'; refusing to treat end-effector joints as arm joints",
                             gripper_type.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        } else if (linkerhand::isSupported(get_hardware_parameter("gripper_type", std::string{})) ||
                   get_hardware_parameter("gripper_type", std::string{}) == "erg32" ||
                   get_hardware_parameter("gripper_type", std::string{}) == "ERG32") {
            RCLCPP_ERROR(get_node()->get_logger(), "Multi-joint end effector configured without any hand joints");
            return hardware_interface::CallbackReturn::ERROR;
        }

        try {
            configureLoad(params);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Invalid load configuration: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "Found %d joints in configuration", joint_count);

        // 计算机械臂关节数量（排除夹爪）
        int arm_joint_count = joint_count;
        if (gripper_) {
            arm_joint_count = joint_count - static_cast<int>(gripper_->getJointCount());
        }
        arm_joint_count_ = static_cast<size_t>(arm_joint_count);
        command_filters_.assign(arm_joint_count_, DoubleSFilter{});
        for (auto &filter : command_filters_) {
            filter.v_min = -command_filter_max_velocity_;
            filter.v_max = command_filter_max_velocity_;
            filter.a_min = -command_filter_max_acceleration_;
            filter.a_max = command_filter_max_acceleration_;
            filter.jerk_limit = command_filter_max_jerk_;
            filter.dt = kRealtimeControlDt;
        }

        // 初始化控制模式标志。启动模式只由 ctrl_mode 参数决定，默认 POSITION。
        use_impedance_control_ = false;
        ctrl_mode_ = "POSITION";
        std::fill(joint_impedance_.begin(), joint_impedance_.end(), 0.0);

        declareControlParameters(params);

        RCLCPP_INFO(get_node()->get_logger(), "==============================================");
        RCLCPP_INFO(get_node()->get_logger(), "RokaeHardware initialized successfully");
        RCLCPP_INFO(get_node()->get_logger(), "  Arm IP: %s", arm_ip_.c_str());
        RCLCPP_INFO(get_node()->get_logger(), "  Local IP: %s", local_ip_.c_str());
        RCLCPP_INFO(get_node()->get_logger(), "  Number of joints: %d", joint_count);
        RCLCPP_INFO(get_node()->get_logger(), "  Control mode: %s", ctrl_mode_.c_str());
        RCLCPP_INFO(get_node()->get_logger(), "  CPU affinity: rt=%d, state=%d",
                    rt_cpu_affinity_, state_cpu_affinity_);
        RCLCPP_INFO(get_node()->get_logger(),
                    "  Command filter: %s (v=%.3f rad/s, a=%.3f rad/s^2, j=%.3f rad/s^3)",
                    use_double_s_filter_ ? "double_s" : "off",
                    command_filter_max_velocity_,
                    command_filter_max_acceleration_,
                    command_filter_max_jerk_);
        RCLCPP_INFO(get_node()->get_logger(), "==============================================");

        // Loading a sibling must not change a robot's mode or start its UDP stream.
        // A later activation error can abort controller_manager without destructors.
        startup_group_ = get_hardware_parameter("startup_group", std::string("default"));
        registerStartupGroup();
        param_callback_handle_ = get_node()->add_on_set_parameters_callback(
            std::bind(&RokaeHardware::paramCallback, this, std::placeholders::_1));
        RCLCPP_INFO(get_node()->get_logger(), "Hardware loaded; robot connection deferred until activation");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void RokaeHardware::configureLoad(
        const hardware_interface::HardwareComponentInterfaceParams &params) {
        load_ = rokae::Load {};
        load_profile_ = "zero default";

        bool has_override = false;
        const auto mass_it = params.hardware_info.hardware_parameters.find("load_mass");
        if (mass_it != params.hardware_info.hardware_parameters.end()) {
            load_.mass = std::stod(mass_it->second);
            if (!std::isfinite(load_.mass) || load_.mass < 0.0) {
                throw std::invalid_argument("load_mass must be finite and non-negative");
            }
            has_override = true;
        }

        const auto read_array_override =
            [&params, &has_override](const std::string &name, std::array<double, 3> &target) {
                const auto it = params.hardware_info.hardware_parameters.find(name);
                if (it == params.hardware_info.hardware_parameters.end()) {
                    return;
                }

                const std::vector<double> parsed = parseDoubleArrayLoose(it->second, {});
                if (parsed.size() != target.size()) {
                    throw std::invalid_argument(name + " must contain exactly 3 values");
                }

                std::copy(parsed.begin(), parsed.end(), target.begin());
                has_override = true;
            };

        read_array_override("load_cog", load_.cog);
        read_array_override("load_inertia", load_.inertia);

        if (std::any_of(load_.cog.begin(), load_.cog.end(),
                        [](double cog) { return !std::isfinite(cog); })) {
            throw std::invalid_argument("load_cog values must be finite");
        }

        if (std::any_of(load_.inertia.begin(), load_.inertia.end(),
                        [](double inertia) { return !std::isfinite(inertia) || inertia < 0.0; })) {
            throw std::invalid_argument("load_inertia values must be finite and non-negative");
        }

        if (has_override) {
            load_profile_ = "hardware parameter configuration";
        }

        RCLCPP_INFO(get_node()->get_logger(),
                    "Configured load (%s): mass=%.6f kg, cog=[%.6f, %.6f, %.6f] m, "
                    "inertia=[%.6f, %.6f, %.6f] kg*m^2",
                    load_profile_.c_str(),
                    load_.mass,
                    load_.cog[0], load_.cog[1], load_.cog[2],
                    load_.inertia[0], load_.inertia[1], load_.inertia[2]);
    }

    void RokaeHardware::declareControlParameters(
        const hardware_interface::HardwareComponentInterfaceParams &params) {
        const auto default_joint_impedance = [this]() {
            std::vector<double> values(arm_joint_count_, 0.0);
            bool has_valid_current = arm_joint_count_ > 0;
            for (size_t i = 0; i < arm_joint_count_ && i < joint_impedance_.size(); ++i) {
                if (joint_impedance_[i] <= 0.0) {
                    has_valid_current = false;
                    break;
                }
                values[i] = joint_impedance_[i];
            }

            if (has_valid_current) {
                return values;
            }

            const std::array<double, 7> fallback = {200.0, 200.0, 200.0, 200.0, 20.0, 20.0, 20.0};
            for (size_t i = 0; i < values.size() && i < fallback.size(); ++i) {
                values[i] = fallback[i];
            }
            return values;
        }();

        std::vector<double> joint_impedance_param = default_joint_impedance;
        if (auto it = params.hardware_info.hardware_parameters.find("joint_impedance");
            it != params.hardware_info.hardware_parameters.end()) {
            joint_impedance_param = parseDoubleArrayLoose(it->second, default_joint_impedance);
        }

        if (joint_impedance_param.size() != arm_joint_count_) {
            RCLCPP_WARN(get_node()->get_logger(),
                        "joint_impedance parameter size (%zu) does not match arm joint count (%zu), using defaults",
                        joint_impedance_param.size(), arm_joint_count_);
            joint_impedance_param = default_joint_impedance;
        }

        for (size_t i = 0; i < joint_impedance_param.size() && i < joint_impedance_.size(); ++i) {
            if (joint_impedance_param[i] <= 0.0) {
                RCLCPP_WARN(get_node()->get_logger(),
                            "joint_impedance contains non-positive values, using defaults");
                joint_impedance_param = default_joint_impedance;
                break;
            }
        }

        if (!get_node()->has_parameter("joint_impedance")) {
            get_node()->declare_parameter<std::vector<double>>("joint_impedance", joint_impedance_param);
        } else if (get_node()->get_parameter("joint_impedance").get_type() ==
                   rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
            joint_impedance_param =
                get_node()->get_parameter("joint_impedance").get_value<std::vector<double>>();
        }

        for (size_t i = 0; i < joint_impedance_param.size() && i < joint_impedance_.size(); ++i) {
            joint_impedance_[i] = joint_impedance_param[i];
        }

        std::string ctrl_mode = ctrl_mode_;
        if (auto it = params.hardware_info.hardware_parameters.find("ctrl_mode");
            it != params.hardware_info.hardware_parameters.end()) {
            ctrl_mode = normalizeControlMode(it->second);
        }

        if (!get_node()->has_parameter("ctrl_mode")) {
            get_node()->declare_parameter<std::string>("ctrl_mode", ctrl_mode);
        } else if (get_node()->get_parameter("ctrl_mode").get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
            ctrl_mode = normalizeControlMode(get_node()->get_parameter("ctrl_mode").as_string());
        }

        if (!applyControlMode(ctrl_mode)) {
            applyControlMode("POSITION");
        }

        const auto read_hw_double = [&params](const std::string &name, double fallback) {
            const auto it = params.hardware_info.hardware_parameters.find(name);
            if (it == params.hardware_info.hardware_parameters.end()) {
                return fallback;
            }
            try {
                return std::stod(it->second);
            } catch (...) {
                return fallback;
            }
        };
        const double left_tool_torque =
            clampToolParam(read_hw_double("left_tool_torque", 1.0));
        const double left_tool_velocity =
            clampToolParam(read_hw_double("left_tool_velocity", 1.0));
        const double right_tool_torque =
            clampToolParam(read_hw_double("right_tool_torque", 1.0));
        const double right_tool_velocity =
            clampToolParam(read_hw_double("right_tool_velocity", 1.0));

        rcl_interfaces::msg::ParameterDescriptor tool_param_descriptor;
        tool_param_descriptor.dynamic_typing = true;
        tool_param_descriptor.description = "Normalized tool torque/velocity in [0.0, 1.0]";
        if (!get_node()->has_parameter("left_tool_torque")) {
            get_node()->declare_parameter<double>("left_tool_torque", left_tool_torque, tool_param_descriptor);
        }
        if (!get_node()->has_parameter("left_tool_velocity")) {
            get_node()->declare_parameter<double>("left_tool_velocity", left_tool_velocity, tool_param_descriptor);
        }
        if (!get_node()->has_parameter("right_tool_torque")) {
            get_node()->declare_parameter<double>("right_tool_torque", right_tool_torque, tool_param_descriptor);
        }
        if (!get_node()->has_parameter("right_tool_velocity")) {
            get_node()->declare_parameter<double>("right_tool_velocity", right_tool_velocity, tool_param_descriptor);
        }

        RCLCPP_INFO(get_node()->get_logger(),
                    "Dynamic control parameters ready: ctrl_mode=%s, joint_impedance size=%zu",
                    ctrl_mode_.c_str(), joint_impedance_param.size());
    }

    rcl_interfaces::msg::SetParametersResult RokaeHardware::paramCallback(
        const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        if (rt_control_faulted_) {
            result.successful = false;
            result.reason = "Rokae fault is latched; restart the hardware process";
            return result;
        }

        bool ctrl_mode_changed = false;
        bool joint_impedance_changed = false;
        bool tool_dynamics_changed = false;
        std::string next_ctrl_mode = ctrl_mode_;
        std::vector<double> next_joint_impedance;
        const auto read_tool_param = [this](const std::string &name, double fallback) {
            if (!get_node()->has_parameter(name)) {
                return fallback;
            }
            const auto param = get_node()->get_parameter(name);
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                return static_cast<double>(param.as_int());
            }
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                return param.as_double();
            }
            return fallback;
        };
        double next_left_tool_torque = read_tool_param("left_tool_torque", 1.0);
        double next_left_tool_velocity = read_tool_param("left_tool_velocity", 1.0);
        double next_right_tool_torque = read_tool_param("right_tool_torque", 1.0);
        double next_right_tool_velocity = read_tool_param("right_tool_velocity", 1.0);

        for (const auto &param : params) {
            if (param.get_name() == "ctrl_mode") {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                    result.successful = false;
                    result.reason = "ctrl_mode must be a string";
                    return result;
                }

                next_ctrl_mode = normalizeControlMode(param.as_string());
                if (next_ctrl_mode != "POSITION" && next_ctrl_mode != "JOINT_IMPEDANCE") {
                    result.successful = false;
                    result.reason = "ctrl_mode must be POSITION or JOINT_IMPEDANCE";
                    return result;
                }

                ctrl_mode_changed = next_ctrl_mode != ctrl_mode_;
            } else if (param.get_name() == "joint_impedance") {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
                    result.successful = false;
                    result.reason = "joint_impedance must be a double array";
                    return result;
                }

                next_joint_impedance = param.as_double_array();
                if (next_joint_impedance.size() != arm_joint_count_) {
                    result.successful = false;
                    result.reason = "joint_impedance must have one value per arm joint";
                    return result;
                }

                if (std::any_of(next_joint_impedance.begin(), next_joint_impedance.end(),
                                [](double value) { return value <= 0.0; })) {
                    result.successful = false;
                    result.reason = "joint_impedance values must be positive";
                    return result;
                }

                joint_impedance_changed = true;
            } else if (param.get_name() == "left_tool_torque" ||
                       param.get_name() == "left_tool_velocity" ||
                       param.get_name() == "right_tool_torque" ||
                       param.get_name() == "right_tool_velocity") {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                    result.successful = false;
                    result.reason = param.get_name() + " must be a number";
                    return result;
                }

                const double value =
                    (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                        ? static_cast<double>(param.as_int())
                        : param.as_double();
                if (value < 0.0 || value > 1.0) {
                    result.successful = false;
                    result.reason = param.get_name() + " must be in [0.0, 1.0]";
                    return result;
                }

                if (param.get_name() == "left_tool_torque") {
                    next_left_tool_torque = value;
                } else if (param.get_name() == "left_tool_velocity") {
                    next_left_tool_velocity = value;
                } else if (param.get_name() == "right_tool_torque") {
                    next_right_tool_torque = value;
                } else {
                    next_right_tool_velocity = value;
                }
                tool_dynamics_changed = true;
            }
        }

        const bool switching_to_position = ctrl_mode_changed && next_ctrl_mode == "POSITION";
        const bool restart_rt_move =
            rt_control_running_ && (ctrl_mode_changed || (joint_impedance_changed && use_impedance_control_));
        std::unique_lock<std::mutex> control_mode_lock(control_mode_mutex_, std::defer_lock);
        if (restart_rt_move) {
            control_mode_lock.lock();
            rt_move_restart_requested_ = true;
            RCLCPP_INFO(get_node()->get_logger(),
                        "Stopping realtime move before applying Rokae ctrl_mode/joint_impedance change");
            try {
                if (rt_controller_) {
                    rt_controller_->stopMove();
                }
            } catch (const std::exception &e) {
                result.successful = false;
                result.reason = std::string("failed to stop realtime move: ") + e.what();
                rt_move_restart_requested_ = false;
                return result;
            }
        }

        if (switching_to_position && hardware_active_) {
            if (!syncPositionCommandsToCurrentState("switching ctrl_mode to POSITION")) {
                result.successful = false;
                result.reason = "failed to sync position commands before switching to POSITION";
                return result;
            }
        }

        if (joint_impedance_changed) {
            for (size_t i = 0; i < next_joint_impedance.size() && i < joint_impedance_.size(); ++i) {
                joint_impedance_[i] = next_joint_impedance[i];
            }
            RCLCPP_INFO(get_node()->get_logger(), "joint_impedance parameter updated");
        }

        if (!applyControlMode(next_ctrl_mode)) {
            result.successful = false;
            result.reason = "failed to apply ctrl_mode";
            return result;
        }

        if (tool_dynamics_changed && gripper_) {
            const bool is_right_gripper = gripperJointIsRight(gripper_->getJointName());
            const double torque = is_right_gripper ? next_right_tool_torque : next_left_tool_torque;
            const double velocity = is_right_gripper ? next_right_tool_velocity : next_left_tool_velocity;
            gripper_->setToolDynamics(torque, velocity);
            if (dynamic_cast<Erg32Gripper*>(gripper_.get()) == nullptr) gripper_->writeCommandPeriodic();
        }

        if (control_mode_lock.owns_lock()) {
            control_mode_lock.unlock();
        }

        return result;
    }

    bool RokaeHardware::applyControlMode(const std::string &ctrl_mode) {
        const std::string normalized = normalizeControlMode(ctrl_mode);
        if (normalized != "POSITION" && normalized != "JOINT_IMPEDANCE") {
            RCLCPP_WARN(get_node()->get_logger(), "Invalid ctrl_mode value: %s", ctrl_mode.c_str());
            return false;
        }

        ctrl_mode_ = normalized;
        use_impedance_control_ = (ctrl_mode_ == "JOINT_IMPEDANCE");

        RCLCPP_INFO(get_node()->get_logger(), "Rokae ctrl_mode set to: %s", ctrl_mode_.c_str());

        return true;
    }

    bool RokaeHardware::syncPositionCommandsToCurrentState(const std::string &reason) {
        if (!arm_robot_) {
            return false;
        }

        std::vector<double> current_positions(arm_joint_count_, 0.0);
        bool got_positions = false;

        try {
            arm_robot_->updateRobotState(std::chrono::milliseconds(1));
            arm_robot_->getStateData(rokae::RtSupportedFields::jointPos_m, joint_positions_array_);
            for (size_t i = 0; i < arm_joint_count_ && i < joint_positions_array_.size(); ++i) {
                current_positions[i] = joint_positions_array_[i];
            }
            got_positions = true;
        } catch (const std::exception &e) {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Failed to read RT state while %s: %s", reason.c_str(), e.what());
        }

        if (!got_positions) {
            std::error_code ec;
            try {
                const auto joint_pos = arm_robot_->jointPos(ec);
                if (!ec) {
                    for (size_t i = 0; i < arm_joint_count_ && i < joint_pos.size(); ++i) {
                        current_positions[i] = joint_pos[i];
                    }
                    got_positions = true;
                } else {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Failed to read jointPos while %s: %s", reason.c_str(), ec.message().c_str());
                }
            } catch (const std::exception &e) {
                RCLCPP_WARN(get_node()->get_logger(),
                            "jointPos threw while %s: %s", reason.c_str(), e.what());
            }
        }

        if (!got_positions) {
            return false;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t i = 0; i < arm_joint_count_ && i < joint_position_commands_.size(); ++i) {
            joint_positions_[i] = current_positions[i];
            joint_position_commands_[i] = current_positions[i];
            target_joint_position_commands_[i] = current_positions[i];
            prev_joint_position_commands_[i] = current_positions[i];
            next_joint_position_commands_[i] = current_positions[i];
            last_sent_joint_positions_[i] = current_positions[i];
        }
        interpolation_alpha_ = 0.0;
        resetCommandFilters(last_sent_joint_positions_);

        RCLCPP_INFO(get_node()->get_logger(),
                    "Synced position commands to current state while %s", reason.c_str());
        return true;
    }

    void RokaeHardware::resetCommandFilters(const std::vector<double> &positions) {
        const size_t count = std::min(command_filters_.size(), positions.size());
        for (size_t i = 0; i < count; ++i) {
            double velocity = 0.0;
            if (i < joint_velocities_.size()) {
                velocity = joint_velocities_[i];
            }
            command_filters_[i].reset(positions[i], velocity, 0.0);
        }
    }

    hardware_interface::CallbackReturn
    RokaeHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */) {
        if (rt_control_faulted_) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Rokae fault is latched; restart the hardware process before activating");
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_node()->get_logger(), "Activating RokaeHardware...");

        try {
            initializeXCoreSDK();
            std::error_code ec;

            // 上电
            arm_robot_->setPowerState(true, ec);
            if (ec) {
                throw std::runtime_error("Failed to power on arm: " + ec.message());
            }

            RCLCPP_INFO(get_node()->get_logger(), "✅ Arm powered on");

            // 夹爪初始化（在机械臂上电后）
            if (gripper_) {
                if (gripper_->initialize(*arm_robot_)) {
                    RCLCPP_INFO(get_node()->get_logger(), "✅ Gripper powered on");
                } else {
                    RCLCPP_WARN(get_node()->get_logger(), "⚠️ Gripper initialization failed");
                    if (dynamic_cast<Erg32Gripper*>(gripper_.get())) {
                        throw std::runtime_error("Required ERG32 initialization failed");
                    }
                }
            }

            // 从当前机器人位置初始化命令位置，避免突然跳跃
            auto current_pos = arm_robot_->jointPos(ec);
            if (!ec && current_pos.size() == arm_joint_count_ &&
                std::all_of(current_pos.begin(), current_pos.end(), [](double q) { return std::isfinite(q); })) {
                for (size_t i = 0; i < arm_joint_count_; ++i) {
                    joint_positions_array_[i] = current_pos[i];
                    joint_velocities_array_[i] = 0.0;
                    joint_efforts_array_[i] = 0.0;
                    joint_positions_[i] = current_pos[i];
                    joint_position_commands_[i] = current_pos[i];
                    target_joint_position_commands_[i] = current_pos[i];
                    // 初始化插值命令为当前位置
                    prev_joint_position_commands_[i] = current_pos[i];
                    next_joint_position_commands_[i] = current_pos[i];
                    last_sent_joint_positions_[i] = current_pos[i];
                }
                // 插值相关变量会在第一次write调用时初始化
                interpolation_alpha_ = 0.0;
                resetCommandFilters(last_sent_joint_positions_);

                // 打印初始位置（动态格式化）
                std::stringstream ss;
                ss << "Initial joint positions (rad): [";
                for (size_t i = 0; i < arm_joint_count_ && i < current_pos.size(); ++i) {
                    ss << std::fixed << std::setprecision(3) << current_pos[i];
                    if (i < arm_joint_count_ - 1) ss << ", ";
                }
                ss << "]";
                RCLCPP_INFO(get_node()->get_logger(), "%s", ss.str().c_str());
            } else {
                throw std::runtime_error("Failed to read initial arm position: " + ec.message());
            }

            if (rt_control_faulted_) throw std::runtime_error("Realtime state failed during activation");
            hardware_active_ = true;
            RCLCPP_INFO(get_node()->get_logger(), "✅ RokaeHardware activated and ready!");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to activate arm: %s; rolling back startup group '%s'",
                         e.what(), startup_group_.c_str());
            rollbackStartupGroup();
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RokaeHardware::on_deactivate(
        const rclcpp_lifecycle::State & /* previous_state */) {
        RCLCPP_INFO(get_node()->get_logger(), "Deactivating RokaeHardware...");

        shutdownRobot();

        RCLCPP_INFO(get_node()->get_logger(), "✅ RokaeHardware deactivated");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RokaeHardware::on_error(const rclcpp_lifecycle::State &) {
        hardware_active_ = false;
        rt_control_faulted_ = true;
        shutdownRobot();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RokaeHardware::on_shutdown(const rclcpp_lifecycle::State &) {
        shutdownRobot();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RokaeHardware::on_cleanup(const rclcpp_lifecycle::State &) {
        shutdownRobot();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> RokaeHardware::on_export_state_interfaces() {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

        // 为所有关节导出状态接口：位置、速度、力矩
        size_t joint_name_sz = joint_names_.size();
        if (gripper_) {
            joint_name_sz = joint_names_.size() - gripper_->getJointCount();
        }

        for (size_t i = 0; i < joint_name_sz; ++i) {
            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_positions_[i]));

            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));

            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i], hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]));
        }

        // 导出夹爪状态接口（如果配置了夹爪）
        if (gripper_) {
            gripper_->exportStateInterfaces(state_interfaces);
        }

        // 输出日志
        if (gripper_) {
            RCLCPP_INFO(get_node()->get_logger(),
                        "Exported %zu state interfaces (%zu arm joints + %zu tool joints)",
                        state_interfaces.size(), joint_name_sz, gripper_->getJointCount());
        } else {
            RCLCPP_INFO(get_node()->get_logger(),
                        "Exported %zu state interfaces for %zu joints",
                        state_interfaces.size(), joint_names_.size());
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr> RokaeHardware::on_export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

        // 为所有关节导出命令接口：位置命令
        size_t joint_name_sz = joint_names_.size();
        if (gripper_) {
            joint_name_sz = joint_names_.size() - gripper_->getJointCount();
        }

        for (size_t i = 0; i < joint_name_sz; ++i) {
            command_interfaces.push_back(
                std::make_shared<hardware_interface::CommandInterface>(
                    joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]));
        }

        // 导出夹爪命令接口（如果配置了夹爪）
        if (gripper_) {
            gripper_->exportCommandInterfaces(command_interfaces);
        }

        // 输出日志
        if (gripper_) {
            RCLCPP_INFO(get_node()->get_logger(),
                        "Exported %zu command interfaces (%zu arm joints + %zu tool joints)",
                        command_interfaces.size(), joint_name_sz, gripper_->getJointCount());
        } else {
            RCLCPP_INFO(get_node()->get_logger(),
                        "Exported %zu command interfaces for %zu joints",
                        command_interfaces.size(), joint_names_.size());
        }

        return command_interfaces;
    }

    hardware_interface::return_type RokaeHardware::read(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/) {
        if (rt_control_faulted_) return hardware_interface::return_type::ERROR;

        // 从缓存的状态数据获取关节信息
        if (arm_robot_ && state_receive_started_) {
            for (size_t i = 0; i < arm_joint_count_; ++i) {
                joint_positions_[i] = joint_positions_array_[i];
                joint_velocities_[i] = joint_velocities_array_[i];
                joint_efforts_[i] = joint_efforts_array_[i];
            }
        }

        // 夹爪状态读取：定期读取（降低读取频率以减少Modbus负载）
        if (gripper_ && gripper_->isInitialized()) {
            const bool ok = gripper_->readStatusPeriodic();
            if (!ok && dynamic_cast<Erg32Gripper*>(gripper_.get())) {
                latchRealtimeFault();
                gripper_->stop();
                return hardware_interface::return_type::ERROR;
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RokaeHardware::write(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/) {
        if (rt_control_faulted_) return hardware_interface::return_type::ERROR;
        if (!hardware_active_) return hardware_interface::return_type::OK;

        // Initial lazy start only: faults remain latched and cannot trigger a restart.
        if (arm_robot_ && !rt_control_running_) {
            startRealtimeControl();
        }
        if (rt_control_faulted_) return hardware_interface::return_type::ERROR;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (target_joint_position_commands_.size() != joint_position_commands_.size()) {
                target_joint_position_commands_.resize(joint_position_commands_.size(), 0.0);
            }

            for (size_t i = 0; i < joint_position_commands_.size(); ++i) {
                target_joint_position_commands_[i] = joint_position_commands_[i];
            }
        }

        // 夹爪控制：检测命令变化并发送
        if (gripper_ && gripper_->isInitialized()) {
            const bool ok = gripper_->writeCommandPeriodic();
            if (!ok && dynamic_cast<Erg32Gripper*>(gripper_.get())) {
                latchRealtimeFault();
                gripper_->stop();
                return hardware_interface::return_type::ERROR;
            }
        }

        return hardware_interface::return_type::OK;
    }

    void RokaeHardware::initializeXCoreSDK() {
        try {
            // 初始化机器人
            arm_robot_ = std::make_unique<rokae::xMateErProRobot>();
            arm_robot_->connectToRobot(arm_ip_, local_ip_);

            RCLCPP_INFO(get_node()->get_logger(), "Connected to robot at %s", arm_ip_.c_str());

            // 设置机器人操作模式为自动模式
            std::error_code ec;
            arm_robot_->setOperateMode(rokae::OperateMode::automatic, ec);
            if (ec) {
                throw std::runtime_error("Failed to set operate mode: " + ec.message());
            }

            // 设置运动控制模式为实时模式
            arm_robot_->setRtNetworkTolerance(100, ec);
            if (ec) throw std::runtime_error("Failed to set realtime network tolerance: " + ec.message());
            arm_robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
            if (ec) {
                throw std::runtime_error("Failed to set motion control mode: " + ec.message());
            }

            // 获取实时控制器
            rt_controller_ = arm_robot_->getRtMotionController().lock();

            if (!rt_controller_) {
                throw std::runtime_error("Failed to get realtime motion controller");
            }

            applyLoad();

            rt_controller_->setFilterLimit(kSdkFilterLimitEnable, kSdkFilterLimit);
            if (kSdkFilterLimitEnable) {
                rt_controller_->setFilterFrequency(
                    kSdkFilterJointFrequency,
                    kSdkFilterCartesianFrequency,
                    kSdkFilterTorqueFrequency,
                    ec);
                if (ec) {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Failed to set SDK filter frequency: %s",
                                ec.message().c_str());
                }
            }

            // 初始化实时控制状态
            rt_control_running_ = false;
            rt_move_restart_requested_ = false;
            state_receive_started_ = false;
            state_update_running_ = false;

            // 启动状态接收
            try {
                arm_robot_->startReceiveRobotState(
                    std::chrono::milliseconds(1),
                    {
                        rokae::RtSupportedFields::jointPos_m,
                        rokae::RtSupportedFields::jointVel_m,
                        rokae::RtSupportedFields::tau_m
                    }
                );

                state_receive_started_ = true;
                RCLCPP_INFO(get_node()->get_logger(), "Robot state receiving started");

                // 等待状态接收稳定
                std::this_thread::sleep_for(std::chrono::milliseconds(200));

                // 启动状态更新线程
                startStateUpdate();
            } catch (const std::exception &e) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to start robot state receiving: %s", e.what());
                throw;
            }

            RCLCPP_INFO(get_node()->get_logger(), "xCoreSDK initialized for arm: %s", arm_ip_.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize xCoreSDK: %s", e.what());
            throw;
        }
    }

    void RokaeHardware::applyLoad() {
        std::error_code ec;
        rt_controller_->setLoad(load_, ec);
        if (ec) {
            throw std::runtime_error("Failed to set robot load: " + ec.message());
        }

        RCLCPP_INFO(get_node()->get_logger(), "Applied robot load profile: %s", load_profile_.c_str());
    }

    void RokaeHardware::startRealtimeControl() {
        if (!hardware_active_ || !rt_controller_ || rt_control_running_ || rt_control_faulted_) {
            return;
        }

        if (rt_control_thread_.joinable()) {
            rt_control_thread_.join();
        }

        try {
            // 启动实时控制线程
            rt_control_running_ = true;
            rt_move_restart_requested_ = false;
            rt_controller_->setLoopRecvDataTimeout(8);
            rt_control_thread_ = std::thread(&RokaeHardware::controlLoop, this);
            bindThreadToCpu(rt_control_thread_, rt_cpu_affinity_, "realtime control");

            RCLCPP_INFO(get_node()->get_logger(), "Realtime control started");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to start realtime control: %s", e.what());
            latchRealtimeFault();
        }
    }

    void RokaeHardware::latchRealtimeFault() {
        // Publish the fault before clearing running, so write() never sees a restartable gap.
        rt_control_faulted_ = true;
        rt_control_running_ = false;
        rt_move_restart_requested_ = false;
        if (rt_controller_) {
            try {
                rt_controller_->stopMove();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to stop faulted realtime move: %s", e.what());
            }
        }
    }

    void RokaeHardware::stopRealtimeControl() {
        // A finished-but-unjoined control thread can still leave the controller-side
        // realtime move active. Treat either condition as a session that must be stopped.
        if (!rt_control_running_ && !rt_control_thread_.joinable()) {
            return;
        }

        rt_control_running_ = false;
        rt_move_restart_requested_ = false;

        // 先停止SDK实时运动，释放controlLoop中的startLoop(true)，再等待线程退出。
        if (rt_controller_) {
            std::lock_guard<std::mutex> lock(control_mode_mutex_);
            try {
                rt_controller_->stopMove();
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Error stopping realtime move: %s", e.what());
            }
        }

        if (rt_controller_) {
            try {
                rt_controller_->stopLoop();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_node()->get_logger(), "Error stopping SDK scheduler: %s", e.what());
            }
        }

        if (rt_control_thread_.joinable()) {
            rt_control_thread_.join();
        }

        RCLCPP_INFO(get_node()->get_logger(), "Realtime control stopped");
    }

    void RokaeHardware::shutdownRobot() {
        hardware_active_ = false;
        // Stop producers first, including finished-but-unjoined workers. Never let
        // a tool transfer delay cancellation of the arm's realtime command loop.
        stopRealtimeControl();
        stopStateUpdate();
        if (gripper_) gripper_->stop();
        if (!arm_robot_) return;

        // Every cleanup step is independent: a failed stop/Idle RPC must not skip
        // power-off, closing UDP ports, or disconnecting the SDK session.
        const auto cleanup = [this](const char* label, const auto& action) {
            try {
                action();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_node()->get_logger(), "%s for arm %s failed: %s",
                             label, arm_ip_.c_str(), e.what());
            }
        };
        if (state_receive_started_.exchange(false)) {
            cleanup("Stop robot state receiving", [this] {
                arm_robot_->stopReceiveRobotState();
                RCLCPP_INFO(get_node()->get_logger(), "Robot state receiving stopped");
            });
        }
        cleanup("Set arm Idle", [this] {
            std::error_code ec;
            arm_robot_->setMotionControlMode(rokae::MotionControlMode::Idle, ec);
            if (ec) throw std::runtime_error(ec.message());
        });
        cleanup("Power off arm", [this] {
            std::error_code ec;
            arm_robot_->setPowerState(false, ec);
            if (ec) throw std::runtime_error(ec.message());
            RCLCPP_INFO(get_node()->get_logger(), "✅ Arm powered off");
        });
        if (rt_controller_) {
            rt_controller_->disconnectNetwork();
            rt_controller_.reset();
        }
        cleanup("Disconnect arm", [this] {
            std::error_code ec;
            arm_robot_->disconnectFromRobot(ec);
            if (ec) throw std::runtime_error(ec.message());
        });
        arm_robot_.reset();
    }

    void RokaeHardware::controlLoop() {
        try {
            // 设置控制回调（1kHz执行）
            const std::function callback = [this] {
                // 读取机器人状态
                arm_robot_->getStateData(rokae::RtSupportedFields::jointPos_m, joint_positions_array_);
                arm_robot_->getStateData(rokae::RtSupportedFields::jointVel_m, joint_velocities_array_);
                arm_robot_->getStateData(rokae::RtSupportedFields::tau_m, joint_efforts_array_);

                std::vector<double> command_positions;
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    if (target_joint_position_commands_.size() < arm_joint_count_) {
                        target_joint_position_commands_.resize(arm_joint_count_, 0.0);
                    }

                    if (use_double_s_filter_) {
                        for (size_t i = 0; i < arm_joint_count_ &&
                                           i < command_filters_.size() &&
                                           i < target_joint_position_commands_.size(); ++i) {
                            last_sent_joint_positions_[i] =
                                command_filters_[i].updateTo(target_joint_position_commands_[i]);
                        }
                    } else {
                        for (size_t i = 0; i < arm_joint_count_ &&
                                           i < target_joint_position_commands_.size(); ++i) {
                            last_sent_joint_positions_[i] = target_joint_position_commands_[i];
                        }
                    }

                    command_positions.assign(
                        last_sent_joint_positions_.begin(),
                        last_sent_joint_positions_.begin() + static_cast<std::ptrdiff_t>(arm_joint_count_));
                }

                rokae::JointPosition command(command_positions);
                // Covers cancellation between the startLoop guard and entering the
                // SDK scheduler, so a late scheduler cannot send an endless hold.
                if (!hardware_active_ || !rt_control_running_ || rt_control_faulted_) command.setFinished();
                return command;
            };

            rt_controller_->setControlLoop(callback, 50, true);

            while (rt_control_running_) {
                {
                    std::lock_guard<std::mutex> lock(control_mode_mutex_);
                    rt_move_restart_requested_ = false;

                    const auto start_move_for_current_mode = [this]() {
                        // 根据配置选择控制模式
                        if (use_impedance_control_) {
                            // 阻抗控制模式：先设置阻抗参数，再启动阻抗控制
                            std::error_code ec;
                            rt_controller_->setJointImpedance(joint_impedance_, ec);
                            if (ec) {
                                RCLCPP_ERROR(get_node()->get_logger(),
                                             "Failed to set joint impedance: %s, falling back to position control",
                                             ec.message().c_str());
                                rt_controller_->startMove(rokae::RtControllerMode::jointPosition);
                            } else {
                                RCLCPP_INFO(get_node()->get_logger(), "Starting joint impedance control");
                                rt_controller_->startMove(rokae::RtControllerMode::jointImpedance);
                            }
                        } else {
                            // 位置控制模式
                            rt_controller_->startMove(rokae::RtControllerMode::jointPosition);
                        }
                    };

                    constexpr int kMaxStartMoveAttempts = 5;
                    bool start_move_succeeded = false;
                    for (int attempt = 1; attempt <= kMaxStartMoveAttempts; ++attempt) {
                        if (!rt_control_running_ || rt_control_faulted_) return;
                        try {
                            start_move_for_current_mode();
                            start_move_succeeded = true;
                            break;
                        } catch (const std::exception &e) {
                            if (attempt == kMaxStartMoveAttempts) {
                                throw;
                            }

                            RCLCPP_WARN(get_node()->get_logger(),
                                        "startMove rejected during startup/mode change (attempt %d/%d): %s",
                                        attempt, kMaxStartMoveAttempts, e.what());
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }
                    }

                    if (!start_move_succeeded) {
                        throw std::runtime_error("Failed to start realtime move");
                    }
                }

                if (!rt_control_running_ || rt_control_faulted_) break;
                try {
                    rt_controller_->startLoop(true);
                } catch (const std::exception &e) {
                    if (rt_control_running_ && rt_move_restart_requested_) {
                        RCLCPP_WARN(get_node()->get_logger(),
                                    "Realtime loop interrupted during requested ctrl_mode change: %s",
                                    e.what());
                    } else {
                        throw;
                    }
                }

                if (!rt_control_running_) {
                    break;
                }

                if (rt_move_restart_requested_) {
                    RCLCPP_INFO(get_node()->get_logger(),
                                "Realtime move stopped for ctrl_mode/joint_impedance change, restarting move");
                    continue;
                }

                RCLCPP_WARN(get_node()->get_logger(), "Realtime move stopped unexpectedly");
                latchRealtimeFault();
                break;
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Control loop error: %s", e.what());
            if (rt_control_running_) latchRealtimeFault();
        }
    }

    void RokaeHardware::startStateUpdate() {
        if (state_update_running_) {
            return;
        }

        try {
            state_update_running_ = true;
            state_update_thread_ = std::thread(&RokaeHardware::stateUpdateLoop, this);
            bindThreadToCpu(state_update_thread_, state_cpu_affinity_, "state update");
            RCLCPP_INFO(get_node()->get_logger(), "State update thread started");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to start state update thread: %s", e.what());
            state_update_running_ = false;
            throw;
        }
    }

    void RokaeHardware::stopStateUpdate() {
        if (!state_update_running_ && !state_update_thread_.joinable()) {
            return;
        }

        state_update_running_ = false;

        if (state_update_thread_.joinable()) {
            state_update_thread_.join();
        }

        RCLCPP_INFO(get_node()->get_logger(), "State update thread stopped");
    }

    void RokaeHardware::stateUpdateLoop() {
        while (state_update_running_ && arm_robot_) {
            try {
                // 更新机器人状态
                arm_robot_->updateRobotState(std::chrono::microseconds(100));
            } catch (const std::exception &e) {
                RCLCPP_ERROR_THROTTLE(
                    get_node()->get_logger(),
                    *get_node()->get_clock(),
                    1000,
                    "State update error: %s", e.what()
                );
                if (state_update_running_) latchRealtimeFault();
                state_update_running_ = false;
                return;
            }

            // 避免过度占用CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20Hz
        }
    }

    void RokaeHardware::bindThreadToCpu(std::thread &thread, int cpu_id, const std::string &thread_name) {
        if (cpu_id < 0) {
            return;
        }

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(static_cast<unsigned int>(cpu_id), &cpuset);

        const int rc = pthread_setaffinity_np(thread.native_handle(), sizeof(cpu_set_t), &cpuset);
        if (rc != 0) {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Failed to bind %s thread for arm %s to CPU %d: %s",
                        thread_name.c_str(), arm_ip_.c_str(), cpu_id, std::strerror(rc));
            return;
        }

        RCLCPP_INFO(get_node()->get_logger(),
                    "Bound %s thread for arm %s to CPU %d",
                    thread_name.c_str(), arm_ip_.c_str(), cpu_id);
    }
} // namespace rokae_ros2_control

// 导出插件
PLUGINLIB_EXPORT_CLASS(rokae_ros2_control::RokaeHardware, hardware_interface::SystemInterface)
