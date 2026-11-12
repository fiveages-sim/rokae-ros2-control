#pragma once

#include "rokae_ros2_control/rokae_gripper_interface.h"

#include "rokae/robot.h"
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>

namespace rokae_ros2_control {

/**
 * @brief Jodell电爪控制类
 * 
 * 封装Jodell电爪的所有参数和控制逻辑，通过Modbus RTU协议与电爪通信
 */
class JodellGripper : public RokaeGripperInterface {
public:
    /**
     * @brief 静态方法：检测关节配置中是否包含夹爪
     * @param joints 关节信息列表
     * @return 夹爪关节名称，如果未检测到则返回空字符串
     */
    static std::string detectGripperJoint(const std::vector<hardware_interface::ComponentInfo>& joints);

    /**
     * @brief 构造函数
     * @param logger ROS2日志记录器
     * @param clock ROS2时钟（用于限流日志）
     * @param joint_name 夹爪关节名称（如果为空字符串，则表示未配置夹爪）
     */
    JodellGripper(
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock,
        const std::string& joint_name = "",
        double normalized_torque = 1.0,
        double normalized_velocity = 1.0);

    /**
     * @brief 析构函数，清理后台线程
     */
    ~JodellGripper() override;
    void stop() override;
    bool communicationStopped() const override { return !command_thread_.joinable() && robot_ptr_ == nullptr; }

    /**
     * @brief 初始化夹爪
     * @param robot 机器人对象
     * @return 是否初始化成功
     */
    bool initialize(rokae::xMateErProRobot& robot) override;

    /**
     * @brief 更新夹爪状态（读取）- 在主循环中调用
     * 注意：实际读取操作在后台线程中异步执行，这里只返回成功
     * @return 是否成功读取
     */
    bool readStatusPeriodic() override;

    /**
     * @brief 处理夹爪命令（写入）- 在主循环中调用，检测命令变化
     * 注意：实际写入操作在后台线程中异步执行
     * @return 是否发送了命令
     */
    bool writeCommandPeriodic() override;

    // 访问器方法
    bool hasGripper() const override { return !joint_name_.empty(); }
    const std::string& getJointName() const override { return joint_name_; }
    bool isInitialized() const override { return initialized_.load(std::memory_order_acquire); }

    // 状态数据指针（用于ROS2 Control接口）
    // 注意：position 和 position_command 现在使用物理位置（米）而不是归一化值
    double* getPositionPtr() override { return &physical_position_; }
    double* getVelocityPtr() override { return &velocity_; }
    double* getEffortPtr() override { return &effort_; }
    double* getPositionCommandPtr() override { return &position_command_; }
    void setToolDynamics(double normalized_torque, double normalized_velocity) override;

    /**
     * @brief 导出夹爪状态接口到硬件接口列表
     * @param state_interfaces 状态接口列表（会被修改，添加夹爪接口）
     */
    void exportStateInterfaces(std::vector<hardware_interface::StateInterface::ConstSharedPtr>& state_interfaces) override;

    /**
     * @brief 导出夹爪命令接口到硬件接口列表
     * @param command_interfaces 命令接口列表（会被修改，添加夹爪接口）
     */
    void exportCommandInterfaces(std::vector<hardware_interface::CommandInterface::SharedPtr>& command_interfaces) override;

private:
    friend class Rs485LifecycleTestPeer;
    // 日志相关
    rclcpp::Logger logger_;         // ROS2日志记录器
    rclcpp::Clock::SharedPtr clock_; // ROS2时钟（用于限流日志）

    // 夹爪配置
    std::string joint_name_;                 // 夹爪关节名称（如果为空则表示未配置夹爪）
    int command_torque_ = 255;               // 命令力矩原始字节值 (0-255)
    int command_velocity_ = 100;             // 命令速度原始字节值 (0-255)
    std::mutex dynamics_mutex_;

    // 夹爪状态
    double position_ = 0.0;                  // 夹爪位置 (0.0-1.0，内部归一化值)
    double physical_position_ = 0.0;         // 夹爪物理位置 (米，用于hardware interface反馈)
    double velocity_ = 0.0;                  // 夹爪速度状态（设备原始字节值 0-255）
    double effort_ = 0.0;                    // 夹爪力状态（设备原始字节值 0-255）
    double position_command_ = 0.0;          // 夹爪位置命令 (米，用于hardware interface输入)
    double last_command_ = 0.0;              // 上一次夹爪命令 (米)
    bool stopped_ = true;                    // 夹爪是否停止
    bool dynamics_dirty_ = false;            // 速度/力矩参数是否需要重发
    uint8_t status_byte_ = 0;                // 状态字节
    uint8_t error_code_ = 0;                 // 错误码字节
    std::atomic<bool> initialized_ = false;  // 夹爪是否已初始化（跨线程发布robot_ptr_）
    
    // 物理参数
    static constexpr double MAX_OPENING_DISTANCE = 0.038372;  // 最大张开距离（米）

    // 命令队列相关
    struct Command {
        int pos_set;
        int trq_set;
        int vel_set;
    };
    std::queue<Command> command_queue_;      // 命令队列
    std::mutex queue_mutex_;                 // 队列互斥锁
    std::condition_variable queue_cv_;       // 队列条件变量
    std::thread command_thread_;              // 后台命令处理线程
    std::atomic<bool> thread_running_ = false;  // 线程运行标志
    rokae::xMateErProRobot* robot_ptr_ = nullptr;  // 机器人对象指针（用于后台线程）
    
    // 状态读取及Modbus事务调度（异步，15Hz）
    struct StatusSnapshot {
        double position = 0.0;
        double physical_position = 0.0;
        double velocity = 0.0;
        double effort = 0.0;
        bool stopped = true;
        uint8_t status_byte = 0;
        uint8_t error_code = 0;
    };

    std::mutex status_mutex_;                // 状态数据互斥锁
    StatusSnapshot background_status_;       // 仅由通信线程更新的状态快照
    bool background_status_valid_ = false;
    static constexpr auto COMMUNICATION_INTERVAL = std::chrono::microseconds(66667);
    static constexpr int INIT_STATUS_READ_ATTEMPTS = 3;
    static constexpr int INIT_POLL_INTERVAL_MS = 100;
    static constexpr int INIT_TIMEOUT_MS = 10000;

    // Modbus RTU 通信参数
    static constexpr int SLAVE_ADDRESS = 9;           // 从站地址
    static constexpr int INIT_FUNC_CODE = 0x10;      // 初始化功能码（写）
    static constexpr int INIT_REG_ADDR = 0x03E8;     // 初始化寄存器地址
    static constexpr int STATUS_FUNC_CODE = 0x04;    // 状态读取功能码（读）
    static constexpr int STATUS_REG_ADDR = 0x07D0;   // 状态寄存器地址

    // 内部辅助方法 - Modbus RTU 通信
    /**
     * @brief Jodell电爪初始化（通过Modbus RTU）
     */
    bool gripInit(rokae::xMateErProRobot& robot);

    /**
     * @brief Jodell电爪运动控制（通过Modbus RTU）
     * @param robot 机器人对象
     * @param trq_set 力矩设置 (0-255)
     * @param vel_set 速度设置 (0-255)
     * @param pos_set 位置设置 (0-255)
     */
    void gripMove(rokae::xMateErProRobot& robot, int trq_set, int vel_set, int pos_set);

    /**
     * @brief 获取Jodell电爪初始化状态
     * @param robot 机器人对象
     * @param init_status 输出初始化状态
     * @return 是否初始化完成
     */
    bool gripGetInitStatus(rokae::xMateErProRobot& robot, uint8_t& init_status, bool& init_finished);

    /**
     * @brief 获取Jodell电爪状态
     * @param robot 机器人对象
     * @param robot_stats 输出状态数据 [状态, 位置/错误, 力/速度]
     */
    bool gripGetStatus(rokae::xMateErProRobot& robot, std::vector<int>& robot_stats);

    // 内部辅助方法 - 状态处理
    void updateStatusFromRegisters(const std::vector<int>& robot_stats);
    bool copyStatusSnapshot();
    bool readStatusInternal(rokae::xMateErProRobot& robot);
    void sendCommand(rokae::xMateErProRobot& robot, int pos_set, int trq_set, int vel_set);
    
    // 后台线程处理函数
    void commandThreadFunction();
    
    // 后台线程状态读取函数
    void readStatusInBackground();
};

} // namespace rokae_ros2_control
