#include "rokae_ros2_control/rokae_hardware.h"

#include <gtest/gtest.h>

namespace rokae_ros2_control {

// Exercise the fault path without creating a robot connection or issuing SDK commands.
class RokaeHardwareTestPeer {
public:
    static void fault(RokaeHardware& hardware) { hardware.latchRealtimeFault(); }
    static void start(RokaeHardware& hardware) { hardware.startRealtimeControl(); }
    static void stop(RokaeHardware& hardware) { hardware.stopRealtimeControl(); }
    static bool running(const RokaeHardware& hardware) { return hardware.rt_control_running_; }
    static bool joinable(const RokaeHardware& hardware) { return hardware.rt_control_thread_.joinable(); }
    static bool restartRequested(const RokaeHardware& hardware) { return hardware.rt_move_restart_requested_; }
    static rcl_interfaces::msg::SetParametersResult changeMode(RokaeHardware& hardware) {
        return hardware.paramCallback({rclcpp::Parameter("ctrl_mode", "POSITION")});
    }
    static void requestModeChange(RokaeHardware& hardware) {
        hardware.rt_control_running_ = true;
        hardware.rt_move_restart_requested_ = true;
    }
};

TEST(RokaeFault, RepeatedWritesReportErrorWithoutRestartingControl) {
    RokaeHardware hardware;
    RokaeHardwareTestPeer::fault(hardware);
    for (int cycle = 0; cycle < 100; ++cycle) {
        EXPECT_EQ(hardware.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.002)),
                  hardware_interface::return_type::ERROR);
        EXPECT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.002)),
                  hardware_interface::return_type::ERROR);
        // A direct start must also be rejected before accessing the absent SDK controller.
        RokaeHardwareTestPeer::start(hardware);
        EXPECT_FALSE(RokaeHardwareTestPeer::running(hardware));
        EXPECT_FALSE(RokaeHardwareTestPeer::joinable(hardware));
    }
}

TEST(RokaeFault, FaultCancelsPendingModeRestartAndSurvivesStop) {
    RokaeHardware hardware;
    RokaeHardwareTestPeer::requestModeChange(hardware);
    RokaeHardwareTestPeer::fault(hardware);
    EXPECT_FALSE(RokaeHardwareTestPeer::running(hardware));
    EXPECT_FALSE(RokaeHardwareTestPeer::restartRequested(hardware));
    RokaeHardwareTestPeer::stop(hardware);
    RokaeHardwareTestPeer::start(hardware);
    EXPECT_FALSE(RokaeHardwareTestPeer::running(hardware));
    EXPECT_EQ(hardware.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.002)),
              hardware_interface::return_type::ERROR);
    const auto change = RokaeHardwareTestPeer::changeMode(hardware);
    EXPECT_FALSE(change.successful);
    EXPECT_NE(change.reason.find("fault is latched"), std::string::npos);
}

} // namespace rokae_ros2_control
