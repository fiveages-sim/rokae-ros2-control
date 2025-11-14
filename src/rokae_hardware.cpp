// Copyright 2024 Rokae Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rokae_ros2_control/rokae_hardware.h"
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
#include <iomanip>
#include <type_traits>
#include <chrono>
#include <algorithm>

namespace rokae_ros2_control {
    hardware_interface::CallbackReturn RokaeHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
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
        const double default_period = 0.01;  // 默认100Hz
        alpha_increment_ = 0.001 / default_period;  // 预计算alpha增量
        interpolation_alpha_ = 0.0;  // 初始插值系数为0

        RCLCPP_INFO(get_node()->get_logger(), "Found %d joints in configuration", joint_count);

        // 读取配置参数的辅助函数（直接从 params 读取）
        // 支持不同类型的参数读取，自动进行类型转换
        const auto get_hardware_parameter = [&params](const std::string &parameter_name, const auto &default_value) {
            if (auto it = params.hardware_info.hardware_parameters.find(parameter_name); it != params.hardware_info.hardware_parameters.end()) {
                if constexpr (std::is_same_v<std::decay_t<decltype(default_value)>, std::string>) {
                    return it->second;
                } else if constexpr (std::is_same_v<std::decay_t<decltype(default_value)>, bool>) {
                    return (it->second == "true" || it->second == "1");
                } else if constexpr (std::is_integral_v<std::decay_t<decltype(default_value)>>) {
                    return static_cast<std::decay_t<decltype(default_value)>>(std::stoi(it->second));
                } else if constexpr (std::is_floating_point_v<std::decay_t<decltype(default_value)>>) {
                    return static_cast<std::decay_t<decltype(default_value)>>(std::stod(it->second));
                }
            }
            return default_value;
        };

        // 读取配置参数
        arm_ip_ = get_hardware_parameter("arm_ip", std::string("192.168.2.161"));
        local_ip_ = get_hardware_parameter("local_ip", std::string("192.168.2.233"));

        // 读取阻抗配置参数
        use_impedance_control_ = false;
        std::fill(joint_impedance_.begin(), joint_impedance_.end(), 0.0);
        
        // 尝试从硬件参数中读取阻抗配置（格式：逗号分隔的数值，如 "500,500,500,500,50,50,50"）
        if (auto it = params.hardware_info.hardware_parameters.find("joint_impedance"); 
            it != params.hardware_info.hardware_parameters.end() && !it->second.empty()) {
            std::stringstream ss(it->second);
            std::string token;
            std::vector<double> impedance_values;
            
            // 解析逗号分隔的数值
            while (std::getline(ss, token, ',')) {
                try {
                    double value = std::stod(token);
                    impedance_values.push_back(value);
                } catch (const std::exception&) {
                    RCLCPP_WARN(get_node()->get_logger(), 
                               "Invalid impedance value in token: %s", token.c_str());
                }
            }
            
            // 验证阻抗参数有效性：数量必须匹配关节数量，且所有值必须大于0
            if (impedance_values.size() == static_cast<size_t>(joint_count)) {
                bool all_positive = true;
                for (size_t i = 0; i < impedance_values.size(); ++i) {
                    if (impedance_values[i] <= 0.0) {
                        all_positive = false;
                        break;
                    }
                    joint_impedance_[i] = impedance_values[i];
                }
                
                if (all_positive) {
                    use_impedance_control_ = true;
                    // 格式化输出阻抗系数
                    std::stringstream ss;
                    ss << "Impedance control enabled with coefficients: [";
                    for (int i = 0; i < joint_count; ++i) {
                        ss << std::fixed << std::setprecision(1) << joint_impedance_[i];
                        if (i < joint_count - 1) ss << ", ";
                    }
                    ss << "]";
                    RCLCPP_INFO(get_node()->get_logger(), "%s", ss.str().c_str());
                } else {
                    RCLCPP_WARN(get_node()->get_logger(), 
                               "Impedance values must be positive, using position control mode");
                }
            } else {
                RCLCPP_WARN(get_node()->get_logger(), 
                           "Impedance parameter count (%zu) doesn't match joint count (%d), using position control mode",
                           impedance_values.size(), joint_count);
            }
        }

        RCLCPP_INFO(get_node()->get_logger(), "==============================================");
        RCLCPP_INFO(get_node()->get_logger(), "RokaeHardware initialized successfully");
        RCLCPP_INFO(get_node()->get_logger(), "  Arm IP: %s", arm_ip_.c_str());
        RCLCPP_INFO(get_node()->get_logger(), "  Local IP: %s", local_ip_.c_str());
        RCLCPP_INFO(get_node()->get_logger(), "  Number of joints: %d", joint_count);
        RCLCPP_INFO(get_node()->get_logger(), "  Control mode: %s", 
                    use_impedance_control_ ? "Impedance" : "Position");
        RCLCPP_INFO(get_node()->get_logger(), "==============================================");

        // 初始化xCoreSDK
        try {
            initializeXCoreSDK();
            RCLCPP_INFO(get_node()->get_logger(), "✅ xCoreSDK initialized successfully");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize xCoreSDK: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
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

            // 从当前机器人位置初始化命令位置，避免突然跳跃
            auto current_pos = arm_robot_->jointPos(ec);
            if (!ec) {
                for (size_t i = 0; i < joint_names_.size(); ++i) {
                    joint_positions_[i] = current_pos[i];
                    joint_position_commands_[i] = current_pos[i];
                    // 初始化插值命令为当前位置
                    prev_joint_position_commands_[i] = current_pos[i];
                    next_joint_position_commands_[i] = current_pos[i];
                    last_sent_joint_positions_[i] = current_pos[i];
                }
                // 插值相关变量会在第一次write调用时初始化
                const double default_period = 0.01;  // 默认100Hz
                alpha_increment_ = 0.001 / default_period;
                interpolation_alpha_ = 0.0;

                // 打印初始位置（动态格式化）
                std::stringstream ss;
                ss << "Initial joint positions (rad): [";
                for (size_t i = 0; i < joint_names_.size(); ++i) {
                    ss << std::fixed << std::setprecision(3) << current_pos[i];
                    if (i < joint_names_.size() - 1) ss << ", ";
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

        // 停止实时控制
        if (rt_control_running_) {
            stopRealtimeControl();
        }

        // 停止状态更新
        if (state_update_running_) {
            stopStateUpdate();
        }

        // 下电
        if (arm_robot_) {
            try {
                std::error_code ec;

                // 下电
                arm_robot_->setPowerState(false, ec);
                if (ec) {
                    RCLCPP_WARN(get_node()->get_logger(), "Failed to power off arm: %s", ec.message().c_str());
                }

                // 设置运动控制模式为空闲
                arm_robot_->setMotionControlMode(rokae::MotionControlMode::Idle, ec);
                if (ec) {
                    RCLCPP_WARN(get_node()->get_logger(), "Failed to set arm to idle mode: %s", ec.message().c_str());
                }

                RCLCPP_INFO(get_node()->get_logger(), "✅ Arm powered off and set to idle mode");
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to deactivate arm: %s", e.what());
            }
        }

        RCLCPP_INFO(get_node()->get_logger(), "✅ RokaeHardware deactivated");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> RokaeHardware::on_export_state_interfaces() {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

        // 为所有关节导出状态接口：位置、速度、力矩
        for (size_t i = 0; i < joint_names_.size(); ++i) {
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

        RCLCPP_INFO(get_node()->get_logger(),
                    "Exported %zu state interfaces for %zu joints",
                    state_interfaces.size(), joint_names_.size());

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr> RokaeHardware::on_export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

        // 为所有关节导出命令接口：位置命令
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            command_interfaces.push_back(
                std::make_shared<hardware_interface::CommandInterface>(
                    joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]));
        }

        RCLCPP_INFO(get_node()->get_logger(),
                    "Exported %zu command interfaces for %zu joints",
                    command_interfaces.size(), joint_names_.size());

        return command_interfaces;
    }

    hardware_interface::return_type RokaeHardware::read(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/) {
        // 从缓存的状态数据获取关节信息
        if (arm_robot_ && state_receive_started_) {
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                joint_positions_[i] = joint_positions_array_[i];
                joint_velocities_[i] = joint_velocities_array_[i];
                joint_efforts_[i] = joint_efforts_array_[i];
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RokaeHardware::write(
        const rclcpp::Time &time,
        const rclcpp::Duration &period) {
        // 启动实时控制线程（类似Dobot的延迟启动策略）
        if (arm_robot_ && !rt_control_running_) {
            startRealtimeControl();
        }

        // 更新命令（自适应频率）
        // 使用上一次实际下发的关节位置作为插值起点（在1kHz callback中记录）
        // 这样可以避免使用可能有延迟的读取位置
        prev_joint_position_commands_ = last_sent_joint_positions_;
        // 更新下一次命令（来自控制器）
        next_joint_position_commands_ = joint_position_commands_;
        
        // 更新硬件更新周期并预计算alpha增量（从period参数获取）
        double hw_period = period.seconds();
        alpha_increment_ = 0.001 / hw_period;  // 1kHz回调周期 / 硬件更新周期
        
        // 重置插值系数为0（开始新的插值周期）
        interpolation_alpha_ = 0.0;

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
            arm_robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
            if (ec) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to set motion control mode: %s", ec.message().c_str());
            }

            // 获取实时控制器
            rt_controller_ = arm_robot_->getRtMotionController().lock();

            if (!rt_controller_) {
                throw std::runtime_error("Failed to get realtime motion controller");
            }

            // 初始化实时控制状态
            rt_control_running_ = false;
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

        try {
            // 启动实时控制线程
            rt_control_running_ = true;
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

        // 等待线程结束
        if (rt_control_thread_.joinable()) {
            rt_control_thread_.join();
        }

        // 停止运动
        if (rt_controller_) {
            rt_controller_->stopMove();
        }

        RCLCPP_INFO(get_node()->get_logger(), "Realtime control stopped");
    }

    void RokaeHardware::controlLoop() {
        try {
            // 设置控制回调（1kHz执行）
            const std::function callback = [this] {
                // 读取机器人状态
                arm_robot_->getStateData(rokae::RtSupportedFields::jointPos_m, joint_positions_array_);
                arm_robot_->getStateData(rokae::RtSupportedFields::jointVel_m, joint_velocities_array_);
                arm_robot_->getStateData(rokae::RtSupportedFields::tau_m, joint_efforts_array_);
                
                // 计算插值命令
                // 累积插值系数（使用预计算的增量）
                interpolation_alpha_ += alpha_increment_;
                
                // 限制alpha在0.0到1.0之间
                if (interpolation_alpha_ > 1.0) {
                    interpolation_alpha_ = 1.0;
                }
                
                // 线性插值生成1kHz命令，直接写入last_sent_joint_positions_
                for (size_t i = 0; i < joint_names_.size() && 
                     i < prev_joint_position_commands_.size() && 
                     i < next_joint_position_commands_.size(); ++i) {
                    last_sent_joint_positions_[i] = prev_joint_position_commands_[i] + 
                                                    (next_joint_position_commands_[i] - prev_joint_position_commands_[i]) * interpolation_alpha_;
                }
                
                return rokae::JointPosition(last_sent_joint_positions_);
            };

            rt_controller_->setControlLoop(callback, 50, true);

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

            rt_controller_->startLoop(true);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Control loop error: %s", e.what());
        }
    }

    void RokaeHardware::startStateUpdate() {
        if (state_update_running_) {
            return;
        }

        try {
            state_update_running_ = true;
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
        }
    }
} // namespace rokae_ros2_control

// 导出插件
PLUGINLIB_EXPORT_CLASS(rokae_ros2_control::RokaeHardware, hardware_interface::SystemInterface)
