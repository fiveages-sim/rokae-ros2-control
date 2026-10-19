#pragma once

#include "rokae_ros2_control/rokae_gripper_interface.h"
#include "rokae_ros2_control/erg32_protocol.h"
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <limits>
#include <mutex>
#include <thread>

namespace rokae_ros2_control {
class Erg32Gripper : public RokaeGripperInterface {
public:
    struct Config {
        int slave_id = 9;
        bool configure_xpanel = false;
        int read_interval_ms = 100;
        int feedback_timeout_ms = 1000;
        int activation_timeout_ms = 10000;
        double normalized_torque = 1.0;
        double normalized_velocity = 1.0;
    };
    Erg32Gripper(rclcpp::Logger logger, std::vector<std::string> joints, Config config);
    ~Erg32Gripper() override;
    bool initialize(rokae::xMateErProRobot& robot) override;
    void stop() override;
    bool readStatusPeriodic() override;
    bool writeCommandPeriodic() override;
    bool hasGripper() const override { return true; }
    const std::string& getJointName() const override { return joints_[0]; }
    size_t getJointCount() const override { return 2; }
    bool isInitialized() const override { return initialized_; }
    double* getPositionPtr() override { return positions_.data(); }
    double* getVelocityPtr() override { return velocities_.data(); }
    double* getEffortPtr() override { return efforts_.data(); }
    double* getPositionCommandPtr() override { return commands_.data(); }
    void setToolDynamics(double torque, double velocity) override;
    void exportStateInterfaces(std::vector<hardware_interface::StateInterface::ConstSharedPtr>& interfaces) override;
    void exportCommandInterfaces(std::vector<hardware_interface::CommandInterface::SharedPtr>& interfaces) override;
private:
    friend class Erg32TestPeer;
    using Clock = std::chrono::steady_clock;
    bool initializeDevice();
    bool readFeedback(erg32::Feedback& feedback);
    void communicationLoop();
    void fail(const char* reason);
    rclcpp::Logger logger_;
    std::array<std::string, 2> joints_;
    Config config_;
    // Only the control thread touches exported storage; the worker publishes a private snapshot.
    std::array<double, 2> positions_{}, velocities_{}, commands_{};
    std::array<double, 2> efforts_{std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
    erg32::Feedback feedback_{};
    Clock::time_point feedback_time_{}, published_time_{};
    erg32::Packet pending_{}, last_{};
    bool command_pending_ = false;
    double torque_, velocity_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::thread worker_;
    std::atomic<bool> running_{false}, faulted_{false};
    bool initialized_ = false;
    // All transfers run serially in activation, worker, or after joining the worker.
    std::function<bool(int, int, std::vector<int>&)> transfer_;
};
} // namespace rokae_ros2_control
