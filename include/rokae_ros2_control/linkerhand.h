#pragma once

#include "rokae_ros2_control/rokae_gripper_interface.h"
#include "rokae_ros2_control/linkerhand_protocol.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rokae_ros2_control {

class LinkerHand : public RokaeGripperInterface {
public:
    struct Config {
        int slave_id = 0x28;
        std::string hand_type = "linkerhand_l6";
        bool right_hand = false;
        bool configure_xpanel = false;
        int read_interval_ms = 100;
        double normalized_torque = 1.0;
        double normalized_velocity = 1.0;
    };

    LinkerHand(
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock,
        std::vector<std::string> joint_names,
        Config config);
    ~LinkerHand() override;
    void stop() override;
    bool communicationStopped() const override { return !communication_thread_.joinable() && robot_ == nullptr; }

    bool initialize(rokae::xMateErProRobot& robot) override;
    bool readStatusPeriodic() override { return initialized_; }
    bool writeCommandPeriodic() override;

    bool hasGripper() const override { return joint_names_.size() == model_.jointCount(); }
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
    friend class Rs485LifecycleTestPeer;
    static constexpr int READ_INPUT_REGS = 0x04;
    static constexpr int WRITE_MULTI_REGS = 0x10;
    static constexpr int POSITION_REG = 0;

    bool readPositions();
    bool writeCommand(const std::vector<double>& command);
    void communicationLoop();
    static double maxFeedbackJump(double upper_limit);

    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
    std::vector<std::string> joint_names_;
    Config config_;
    linkerhand::Model model_;
    std::vector<double> positions_{};
    std::vector<double> velocities_{};
    std::vector<double> efforts_{};
    std::vector<double> commands_{};
    std::vector<double> pending_commands_{};
    std::vector<double> last_commands_{};
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
