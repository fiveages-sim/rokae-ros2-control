#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

// xCoreSDK includes
#include "rokae/robot.h"
#include "rokae/motion_control_rt.h"
#include "rokae_ros2_control/jodell_gripper.h"
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

namespace rokae_ros2_control {
    /**
     * @brief Rokae机器人的ROS2 Control硬件接口
     *
     * 该类实现了hardware_interface::SystemInterface，通过xCoreSDK与Rokae机械臂通信
     * 重构后的设计：一个hardware实例只控制一个机械臂（7自由度）
     * 参考Dobot的设计思路，简化架构，提高模块化程度
     */
    class RokaeHardware : public hardware_interface::SystemInterface {
    public:
        /**
         * @brief 初始化硬件接口
         * @param params 硬件组件接口参数（包含硬件信息、日志记录器等）
         * @return 初始化结果
         */
        hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

        /**
         * @brief 激活硬件接口（连接机器人、上电、初始化实时控制）
         * @param previous_state 前一个生命周期状态
         * @return 激活结果
         */
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief 停用硬件接口（停止实时控制、下电、断开连接）
         * @param previous_state 前一个生命周期状态
         * @return 停用结果
         */
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief 导出状态接口（位置、速度、力矩）
         * @return 状态接口列表（智能指针）
         */
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;

        /**
         * @brief 导出命令接口（位置命令）
         * @return 命令接口列表（智能指针）
         */
        std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

        /**
         * @brief 读取机器人状态
         * @param time 当前时间
         * @param period 距离上次读取的时间间隔
         * @return 读取结果
         */
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /**
         * @brief 写入控制命令到机器人
         * @param time 当前时间
         * @param period 距离上次写入的时间间隔
         * @return 写入结果
         */
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        // 关节数据存储（动态大小，根据配置）
        std::vector<double> joint_positions_; // 关节位置（弧度）
        std::array<double, 7> joint_positions_array_; // 关节位置（弧度）
        std::vector<double> joint_velocities_; // 关节速度（弧度/秒）
        std::array<double, 7> joint_velocities_array_; // 关节速度（弧度）
        std::vector<double> joint_efforts_; // 关节力矩（N·m）
        std::array<double, 7> joint_efforts_array_; // 关节力矩（弧度）
        std::vector<double> joint_position_commands_; // 关节位置命令（弧度）
        std::vector<double> joint_velocities_commands_; // 关节速度命令（弧度）
        std::vector<double> joint_efforts_commands_; // 关节力矩命令（牛顿）
        std::vector<std::string> joint_names_; // 关节名称列表

        // 夹爪控制
        std::unique_ptr<JodellGripper> gripper_; // Jodell电爪控制器

        // 命令插值相关（自适应硬件接口频率到1kHz实时控制的插值）
        std::vector<double> prev_joint_position_commands_; // 上一次关节位置命令（用于插值）
        std::vector<double> next_joint_position_commands_; // 下一次关节位置命令（用于插值）
        std::vector<double> last_sent_joint_positions_; // 上一次实际下发的关节位置（在1kHz callback中记录）
        double alpha_increment_; // 每次1kHz回调的alpha增量（在write中预计算：0.001 / hw_update_period_）
        double interpolation_alpha_; // 插值系数（0.0到1.0，在controlLoop中自动累积，在write中重置）

        // 配置参数
        std::string arm_ip_; // 机械臂IP地址
        std::string local_ip_; // 本地IP地址
        bool use_impedance_control_; // 是否使用阻抗控制模式
        std::array<double, 7> joint_impedance_; // 关节阻抗系数（Nm/rad）
        bool use_servoj_control_; // 是否使用servoj控制模式
        double servoj_period_; // servoj发送周期（秒，应与controller_manager的update_rate匹配）
        double servoj_lookahead_; // servoj前瞻时间（秒）
        double servoj_kp_; // servoj控制增益

        // xCoreSDK相关
        std::unique_ptr<rokae::xMateErProRobot> arm_robot_;
        std::shared_ptr<rokae::RtMotionControlCobot<7> > rt_controller_;

        // 实时控制线程
        std::thread rt_control_thread_;
        std::atomic<bool> rt_control_running_;
        std::mutex data_mutex_;

        // 状态接收相关
        std::atomic<bool> state_receive_started_;
        std::thread state_update_thread_;
        std::atomic<bool> state_update_running_;

        // xCoreSDK相关方法
        void initializeXCoreSDK();

        void startRealtimeControl();

        void stopRealtimeControl();

        void controlLoop();

        // 状态更新相关方法
        void startStateUpdate();

        void stopStateUpdate();

        void stateUpdateLoop();
    };
} // namespace rokae_ros2_control
