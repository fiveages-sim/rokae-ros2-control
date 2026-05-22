#include "rokae_ros2_control/rokae_hardware.h"
#include "rokae_ros2_control/ag2f90d_gripper.h"
#include "rokae_ros2_control/jodell_gripper.h"
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
#include <iomanip>
#include <type_traits>
#include <chrono>
#include <algorithm>
#include <cctype>

namespace rokae_ros2_control {
    namespace {
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
    } // namespace

    RokaeHardware::~RokaeHardware() {
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

        // 初始化插值相关变量
        prev_joint_position_commands_.resize(joint_count, 0.0);
        next_joint_position_commands_.resize(joint_count, 0.0);
        last_sent_joint_positions_.resize(joint_count, 0.0);
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

        const std::string gripper_joint_name = detectGripperJoint(params.hardware_info.joints);
        if (!gripper_joint_name.empty()) {
            std::string gripper_type = get_hardware_parameter("gripper_type", std::string("ag2f90d"));
            std::transform(gripper_type.begin(), gripper_type.end(), gripper_type.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

            RCLCPP_INFO(get_node()->get_logger(),
                        "Detected Rokae gripper joint '%s', gripper_type=%s",
                        gripper_joint_name.c_str(),
                        gripper_type.c_str());

            if (gripper_type == "jodell") {
                gripper_ = std::make_unique<JodellGripper>(
                    get_node()->get_logger(),
                    get_node()->get_clock(),
                    gripper_joint_name);
            } else if (gripper_type == "ag2f90d" || gripper_type == "ag2f90" ||
                       gripper_type == "changingtek90d") {
                Ag2f90dGripper::Config config;
                config.slave_id = get_hardware_parameter("gripper_slave_id", 1);
                config.close_pos = get_hardware_parameter("gripper_close_pos", 9000);
                config.max_opening_distance =
                    get_hardware_parameter("gripper_max_opening_distance", 0.09);
                config.configure_xpanel =
                    get_hardware_parameter("gripper_configure_xpanel", false);
                config.read_interval_ms =
                    get_hardware_parameter("gripper_read_interval_ms", 500);
                gripper_ = std::make_unique<Ag2f90dGripper>(
                    get_node()->get_logger(),
                    get_node()->get_clock(),
                    gripper_joint_name,
                    config);
            } else {
                RCLCPP_WARN(get_node()->get_logger(),
                            "Unknown Rokae gripper_type '%s', continuing without gripper",
                            gripper_type.c_str());
            }
        }

        RCLCPP_INFO(get_node()->get_logger(), "Found %d joints in configuration", joint_count);

        // 计算机械臂关节数量（排除夹爪）
        int arm_joint_count = joint_count;
        if (gripper_) {
            arm_joint_count = joint_count - 1;
        }
        arm_joint_count_ = static_cast<size_t>(arm_joint_count);

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
        RCLCPP_INFO(get_node()->get_logger(), "==============================================");

        // 初始化xCoreSDK
        try {
            initializeXCoreSDK();
            param_callback_handle_ = get_node()->add_on_set_parameters_callback(
                std::bind(&RokaeHardware::paramCallback, this, std::placeholders::_1));
            RCLCPP_INFO(get_node()->get_logger(), "✅ xCoreSDK initialized successfully");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize xCoreSDK: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
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

        RCLCPP_INFO(get_node()->get_logger(),
                    "Dynamic control parameters ready: ctrl_mode=%s, joint_impedance size=%zu",
                    ctrl_mode_.c_str(), joint_impedance_param.size());
    }

    rcl_interfaces::msg::SetParametersResult RokaeHardware::paramCallback(
        const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        bool ctrl_mode_changed = false;
        bool joint_impedance_changed = false;
        std::string next_ctrl_mode = ctrl_mode_;
        std::vector<double> next_joint_impedance;

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

        if (switching_to_position) {
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
            prev_joint_position_commands_[i] = current_positions[i];
            next_joint_position_commands_[i] = current_positions[i];
            last_sent_joint_positions_[i] = current_positions[i];
        }
        interpolation_alpha_ = 0.0;

        RCLCPP_INFO(get_node()->get_logger(),
                    "Synced position commands to current state while %s", reason.c_str());
        return true;
    }

    hardware_interface::CallbackReturn
    RokaeHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */) {
        RCLCPP_INFO(get_node()->get_logger(), "Activating RokaeHardware...");

        if (!arm_robot_) {
            RCLCPP_ERROR(get_node()->get_logger(), "Robot not initialized!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        try {
            std::error_code ec;

            // 上电
            arm_robot_->setPowerState(true, ec);
            if (ec) {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to power on arm: %s", ec.message().c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            RCLCPP_INFO(get_node()->get_logger(), "✅ Arm powered on");

            // 夹爪初始化（在机械臂上电后）
            if (gripper_) {
                if (gripper_->initialize(*arm_robot_)) {
                    RCLCPP_INFO(get_node()->get_logger(), "✅ Gripper powered on");
                } else {
                    RCLCPP_WARN(get_node()->get_logger(), "⚠️ Gripper initialization failed");
                }
            }

            // 从当前机器人位置初始化命令位置，避免突然跳跃
            auto current_pos = arm_robot_->jointPos(ec);
            if (!ec) {
                for (size_t i = 0; i < arm_joint_count_ && i < current_pos.size(); ++i) {
                    joint_positions_[i] = current_pos[i];
                    joint_position_commands_[i] = current_pos[i];
                    // 初始化插值命令为当前位置
                    prev_joint_position_commands_[i] = current_pos[i];
                    next_joint_position_commands_[i] = current_pos[i];
                    last_sent_joint_positions_[i] = current_pos[i];
                }
                // 插值相关变量会在第一次write调用时初始化
                interpolation_alpha_ = 0.0;

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
                RCLCPP_WARN(get_node()->get_logger(), "Failed to read current position: %s", ec.message().c_str());
            }

            RCLCPP_INFO(get_node()->get_logger(), "✅ RokaeHardware activated and ready!");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to activate arm: %s", e.what());
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

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> RokaeHardware::on_export_state_interfaces() {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

        // 为所有关节导出状态接口：位置、速度、力矩
        size_t joint_name_sz = joint_names_.size();
        if (gripper_) {
            joint_name_sz = joint_names_.size() - 1;
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
                        "Exported %zu state interfaces (%zu arm joints + 1 gripper)",
                        state_interfaces.size(), joint_name_sz);
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
            joint_name_sz = joint_names_.size() - 1;
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
                        "Exported %zu command interfaces (%zu arm joints + 1 gripper)",
                        command_interfaces.size(), joint_name_sz);
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
            gripper_->readStatusPeriodic();
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RokaeHardware::write(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration &period) {
        // 启动实时控制线程（类似Dobot的延迟启动策略）
        if (arm_robot_ && !rt_control_running_) {
            startRealtimeControl();
        }

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            // 更新命令（自适应频率）
            // 从当前实际位置（last_sent_joint_positions_）开始新的插值，平滑过渡到新目标。
            prev_joint_position_commands_ = last_sent_joint_positions_;
            next_joint_position_commands_ = joint_position_commands_;

            // 更新硬件更新周期并预计算alpha增量（从period参数获取）
            double hw_period = period.seconds();
            alpha_increment_ = 0.001 / hw_period; // 1kHz回调周期 / 硬件更新周期

            // 重置插值系数为0（开始新的插值周期：从当前位置到新目标）
            interpolation_alpha_ = 0.0;
        }

        // 夹爪控制：检测命令变化并发送
        if (gripper_ && gripper_->isInitialized()) {
            gripper_->writeCommandPeriodic();
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
                RCLCPP_WARN(get_node()->get_logger(), "Failed to set operate mode: %s", ec.message().c_str());
            }

            // 设置运动控制模式为实时模式
            arm_robot_->setRtNetworkTolerance(100, ec);
            arm_robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
            if (ec) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to set motion control mode: %s", ec.message().c_str());
            }

            // 获取实时控制器
            rt_controller_ = arm_robot_->getRtMotionController().lock();

            if (!rt_controller_) {
                throw std::runtime_error("Failed to get realtime motion controller");
            }

            rt_controller_->setFilterLimit(true, 10);
            rt_controller_->setFilterFrequency(10, 10, 10, ec);

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
                state_receive_started_ = false;
            }

            RCLCPP_INFO(get_node()->get_logger(), "xCoreSDK initialized for arm: %s", arm_ip_.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize xCoreSDK: %s", e.what());
            throw;
        }
    }

    void RokaeHardware::startRealtimeControl() {
        if (rt_control_running_) {
            return;
        }

        if (rt_control_thread_.joinable()) {
            rt_control_thread_.join();
        }

        try {
            // 启动实时控制线程
            rt_control_running_ = true;
            rt_move_restart_requested_ = false;
            rt_controller_->setLoopRecvDataTimeout(10);
            rt_control_thread_ = std::thread(&RokaeHardware::controlLoop, this);

            RCLCPP_INFO(get_node()->get_logger(), "Realtime control started");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to start realtime control: %s", e.what());
            rt_control_running_ = false;
        }
    }

    void RokaeHardware::stopRealtimeControl() {
        if (!rt_control_running_) {
            return;
        }

        rt_control_running_ = false;

        // 先停止SDK实时运动，释放controlLoop中的startLoop(true)，再等待线程退出。
        if (rt_controller_) {
            try {
                rt_controller_->stopMove();
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Error stopping realtime move: %s", e.what());
            }
        }

        if (rt_control_thread_.joinable()) {
            rt_control_thread_.join();
        }

        RCLCPP_INFO(get_node()->get_logger(), "Realtime control stopped");
    }

    void RokaeHardware::shutdownRobot() {
        if (!arm_robot_) {
            return;
        }

        try {
            std::error_code ec;
            RCLCPP_INFO(get_node()->get_logger(), "Sending arm power-off command before controller cleanup...");
            arm_robot_->setPowerState(false, ec);
            if (ec) {
                RCLCPP_WARN(get_node()->get_logger(), "Early arm power-off command failed: %s", ec.message().c_str());
            } else {
                RCLCPP_INFO(get_node()->get_logger(), "Early arm power-off command sent");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Early arm power-off command threw: %s", e.what());
        }

        if (rt_control_running_) {
            stopRealtimeControl();
        }

        if (state_update_running_) {
            stopStateUpdate();
        }

        try {
            if (state_receive_started_) {
                arm_robot_->stopReceiveRobotState();
                state_receive_started_ = false;
                RCLCPP_INFO(get_node()->get_logger(), "Robot state receiving stopped");
            }

            std::error_code ec;

            arm_robot_->setMotionControlMode(rokae::MotionControlMode::Idle, ec);
            if (ec) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to set arm to idle mode: %s", ec.message().c_str());
            }

            arm_robot_->setPowerState(false, ec);
            if (ec) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to power off arm: %s", ec.message().c_str());
            } else {
                RCLCPP_INFO(get_node()->get_logger(), "✅ Arm powered off");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to shutdown arm safely: %s", e.what());
        }
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
                    // 计算插值命令
                    // 累积插值系数（使用预计算的增量）
                    interpolation_alpha_ += alpha_increment_;

                    // 限制alpha在0.0到1.0之间
                    if (interpolation_alpha_ > 1.0) {
                        interpolation_alpha_ = 1.0;
                    }

                    // 线性插值生成1kHz命令，直接写入last_sent_joint_positions_
                    for (size_t i = 0; i < arm_joint_count_ &&
                                       i < prev_joint_position_commands_.size() &&
                                       i < next_joint_position_commands_.size(); ++i) {
                        last_sent_joint_positions_[i] = prev_joint_position_commands_[i] +
                                                        (next_joint_position_commands_[i] - prev_joint_position_commands_[
                                                             i]) * interpolation_alpha_;
                    }
                    command_positions.assign(
                        last_sent_joint_positions_.begin(),
                        last_sent_joint_positions_.begin() + static_cast<std::ptrdiff_t>(arm_joint_count_));
                }

                return rokae::JointPosition(command_positions);
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
                        try {
                            start_move_for_current_mode();
                            start_move_succeeded = true;
                            break;
                        } catch (const std::exception &e) {
                            if (attempt == kMaxStartMoveAttempts) {
                                throw;
                            }

                            RCLCPP_WARN(get_node()->get_logger(),
                                        "startMove rejected during ctrl_mode change (attempt %d/%d): %s",
                                        attempt, kMaxStartMoveAttempts, e.what());
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }
                    }

                    if (!start_move_succeeded) {
                        throw std::runtime_error("Failed to start realtime move");
                    }
                }

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
                rt_control_running_ = false;
                break;
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Control loop error: %s", e.what());
            rt_control_running_ = false;
        }
    }

    void RokaeHardware::startStateUpdate() {
        if (state_update_running_) {
            return;
        }

        try {
            state_update_running_ = true;
            state_update_thread_ = std::thread(&RokaeHardware::stateUpdateLoop, this);
            RCLCPP_INFO(get_node()->get_logger(), "State update thread started");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to start state update thread: %s", e.what());
            state_update_running_ = false;
        }
    }

    void RokaeHardware::stopStateUpdate() {
        if (!state_update_running_) {
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
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // 避免过度占用CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20Hz
        }
    }
} // namespace rokae_ros2_control

// 导出插件
PLUGINLIB_EXPORT_CLASS(rokae_ros2_control::RokaeHardware, hardware_interface::SystemInterface)
