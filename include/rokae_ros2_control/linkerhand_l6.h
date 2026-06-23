#pragma once

#include "rokae_ros2_control/rokae_gripper_interface.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rokae_ros2_control {

class LinkerHandL6 : public RokaeGripperInterface {
public:
    struct Config {
        int slave_id = 0x28;
        bool configure_xpanel = false;
        int read_interval_ms = 100;
        double normalized_torque = 1.0;
        double normalized_velocity = 1.0;
    };

    LinkerHandL6(
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock,
        std::vector<std::string> joint_names,
        Config config);
    ~LinkerHandL6() override;

    bool initialize(rokae::xMateErProRobot& robot) override;
    bool readStatusPeriodic() override { return initialized_; }
    bool writeCommandPeriodic() override;

    bool hasGripper() const override { return joint_names_.size() == JOINT_COUNT; }
    const std::string& getJointName() const override { return joint_names_.front(); }
    size_t getJointCount() const override { return joint_names_.size(); }
    bool isInitialized() const override { return initialized_; }

    double* getPositionPtr() override { return positions_.data(); }
    double* getVelocityPtr() override { return velocities_.data(); }
    double* getEffortPtr() override { return efforts_.data(); }
    double* getPositionCommandPtr() override { return commands_.data(); }
    void setToolDynamics(double normalized_torque, double normalized_velocity) override;

    void exportStateInterfaces(
        std::vector<hardware_interface::StateInterface::ConstSharedPtr>& state_interfaces) override;
    void exportCommandInterfaces(
        std::vector<hardware_interface::CommandInterface::SharedPtr>& command_interfaces) override;

private:
    static constexpr size_t JOINT_COUNT = 6;
    static constexpr int READ_INPUT_REGS = 0x04;
    static constexpr int WRITE_MULTI_REGS = 0x10;
    static constexpr int POSITION_REG = 0;
    static constexpr int TORQUE_REG = 6;
    static constexpr int SPEED_REG = 12;
    static constexpr int REGISTERS_PER_REQUEST = 3;

    bool readPositions();
    bool writeCommand(const std::array<double, JOINT_COUNT>& command);
    void communicationLoop();
    static double jointUpperLimit(const std::string& joint_name);
    static double maxFeedbackJump(double upper_limit);
    static int positionToRegister(double position, double upper_limit);
    static double registerToPosition(int value, double upper_limit);

    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
    std::vector<std::string> joint_names_;
    Config config_;
    std::array<double, JOINT_COUNT> positions_{};
    std::array<double, JOINT_COUNT> velocities_{};
    std::array<double, JOINT_COUNT> efforts_{};
    std::array<double, JOINT_COUNT> commands_{};
    std::array<double, JOINT_COUNT> pending_commands_{};
    std::array<double, JOINT_COUNT> last_commands_{};
    bool have_valid_positions_ = false;
    double torque_ = 1.0;
    double velocity_ = 1.0;
    bool dynamics_dirty_ = true;
    bool command_pending_ = false;
    bool initialized_ = false;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::thread communication_thread_;
    std::atomic<bool> running_{false};
    rokae::xMateErProRobot* robot_ = nullptr;
};

} // namespace rokae_ros2_control
