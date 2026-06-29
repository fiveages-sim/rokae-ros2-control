#pragma once

#include "rokae_ros2_control/rokae_gripper_interface.h"

#include <hardware_interface/hardware_info.hpp>
#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rokae_ros2_control {

/** Jodell ERG32-150: rotate + grip (2-DOF) via Rokae end-tool Modbus RTU. */
class Erg32Gripper : public RokaeGripperInterface {
public:
    struct Config {
        double normalized_torque = 1.0;
        double normalized_velocity = 1.0;
    };

    static std::vector<std::string> detectJoints(
        const std::vector<hardware_interface::ComponentInfo>& joints);

    Erg32Gripper(
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock,
        std::vector<std::string> joint_names,
        Config config);
    ~Erg32Gripper() override;

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
    static constexpr size_t JOINT_COUNT = 2;
    static constexpr int REGISTERS_PER_REQUEST = 3;
    static constexpr double kPi = 3.1415926535897931;
    static constexpr int READ_INTERVAL_MS = 500;

    static constexpr int SLAVE_ADDRESS = 9;
    static constexpr int READ_FUNCTION = 0x03;
    static constexpr int WRITE_FUNCTION = 0x10;
    static constexpr uint16_t CTRL_REG_BASE = 0x03E8;
    static constexpr int CTRL_REG_COUNT = 2;
    static constexpr uint16_t GRIP_PARAM_REG = 0x03EA;
    static constexpr uint16_t STATUS_REG_ADDR = 0x07D0;
    static constexpr int STATUS_REG_COUNT = 8;
    static constexpr uint16_t ACTIVATE_VALUE = 0x0001;
    static constexpr uint8_t GRIP_TRIGGER = 0x01;
    static constexpr uint8_t ROTATE_ABS_TRIGGER = 0x01;
    static constexpr double MAX_GRIP_OPENING_M = 0.016;

    bool writeRegisters(uint16_t reg_addr, const std::vector<int>& values);
    bool readStatus();
    bool writeCommand(const std::array<double, JOINT_COUNT>& command);
    void communicationLoop();
    static uint8_t toByte(double normalized);

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
