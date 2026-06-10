#pragma once

#include "rokae_ros2_control/rokae_gripper_interface.h"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

namespace rokae_ros2_control {

class Ag2f90dGripper : public RokaeGripperInterface {
public:
    struct Config {
        int slave_id = 0x01;
        int close_pos = 9000;
        double max_opening_distance = 0.09;
        bool configure_xpanel = false;
        int read_interval_ms = 500;
        int feedback_reg = 0x0418;
        std::string model_name = "AG2F90D";
    };

    Ag2f90dGripper(
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock,
        std::string joint_name,
        Config config);
    ~Ag2f90dGripper() override;

    bool initialize(rokae::xMateErProRobot& robot) override;
    bool readStatusPeriodic() override;
    bool writeCommandPeriodic() override;

    bool hasGripper() const override { return !joint_name_.empty(); }
    const std::string& getJointName() const override { return joint_name_; }
    bool isInitialized() const override { return initialized_; }

    double* getPositionPtr() override { return &position_; }
    double* getVelocityPtr() override { return &velocity_; }
    double* getEffortPtr() override { return &effort_; }
    double* getPositionCommandPtr() override { return &position_command_; }

    void exportStateInterfaces(
        std::vector<hardware_interface::StateInterface::ConstSharedPtr>& state_interfaces) override;
    void exportCommandInterfaces(
        std::vector<hardware_interface::CommandInterface::SharedPtr>& command_interfaces) override;

    static constexpr int FEEDBACK_REG_D = 0x0418;
    static constexpr int FEEDBACK_REG_C = 0x060D;

private:
    struct Command {
        int pos_set;
    };

    static constexpr int READ_HOLDING_REGS = 0x03;
    static constexpr int WRITE_SINGLE_REG = 0x06;
    static constexpr int WRITE_MULTI_REGS = 0x10;
    static constexpr int INIT_REG = 0x0100;
    static constexpr int POS_REG = 0x0102;
    static constexpr int TRIGGER_REG = 0x0108;
    static constexpr int POWER_ON = 0x0001;
    static constexpr int TRIGGER_MOVE = 0x0001;
    static constexpr int OPEN_POS = 0;

    bool gripInit(rokae::xMateErProRobot& robot);
    bool gripMove(rokae::xMateErProRobot& robot, int pos_set);
    bool gripGetPosition(rokae::xMateErProRobot& robot, int& pos_now_get);
    void readStatusInternal(rokae::xMateErProRobot& robot);
    void readStatusInBackground();
    void commandThreadFunction();

    int normalizedToRegister(double normalized_position) const;
    double registerToNormalized(int register_pos) const;

    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
    std::string joint_name_;
    Config config_;

    double position_ = 0.0;
    double velocity_ = 0.0;
    double effort_ = 0.0;
    double position_command_ = 0.0;
    double last_command_ = 0.0;
    bool initialized_ = false;

    std::queue<Command> command_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread command_thread_;
    std::atomic<bool> thread_running_ = false;
    rokae::xMateErProRobot* robot_ptr_ = nullptr;
    std::mutex status_mutex_;
};

} // namespace rokae_ros2_control
