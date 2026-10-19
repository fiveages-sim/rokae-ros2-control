#include "rokae_ros2_control/rokae_hardware.h"

#include <gtest/gtest.h>

namespace rokae_ros2_control {

// Exercise the fault path without creating a robot connection or issuing SDK commands.
class RokaeHardwareTestPeer {
public:
    static bool connected(const RokaeHardware& hardware) { return hardware.arm_robot_ != nullptr; }
    static bool receiving(const RokaeHardware& hardware) { return hardware.state_receive_started_; }
    static bool active(const RokaeHardware& hardware) { return hardware.hardware_active_; }
    static bool faulted(const RokaeHardware& hardware) { return hardware.rt_control_faulted_; }
    static bool stateJoinable(const RokaeHardware& hardware) { return hardware.state_update_thread_.joinable(); }
    static void activateForTest(RokaeHardware& hardware) { hardware.hardware_active_ = true; }
    static void finishedStateWorker(RokaeHardware& hardware) {
        hardware.state_update_thread_ = std::thread([] {});
        hardware.state_update_running_ = false;
    }
    static void rollback(RokaeHardware& hardware) { hardware.rollbackStartupGroup(); }
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

class RokaeStartup : public ::testing::Test {
protected:
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    void SetUp() override {
        rclcpp::init(0, nullptr);
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }
    void TearDown() override { executor_.reset(); rclcpp::shutdown(); }
    hardware_interface::HardwareComponentParams parameters(
        const std::string& name, bool erg32 = false, const std::string& group = "test_pair") {
        hardware_interface::HardwareComponentParams p;
        p.clock = std::make_shared<rclcpp::Clock>();
        p.executor = executor_;
        p.hardware_info.is_async = false;
        p.hardware_info.name = name;
        p.hardware_info.type = "system";
        p.hardware_info.hardware_parameters = {
            {"arm_ip", "203.0.113.1"}, {"local_ip", "203.0.113.2"}, {"startup_group", group}};
        for (int i = 0; i < 7; ++i) {
            hardware_interface::ComponentInfo joint;
            joint.name = name + "_joint" + std::to_string(i);
            p.hardware_info.joints.push_back(joint);
        }
        if (erg32) {
            p.hardware_info.hardware_parameters["gripper_type"] = "erg32";
            for (const auto* name : {"right_hand_rotate_joint", "right_hand_gripper_joint"}) {
                hardware_interface::ComponentInfo joint;
                joint.name = name;
                p.hardware_info.joints.push_back(joint);
            }
        }
        return p;
    }
};

TEST_F(RokaeStartup, LoadingBothArmsDoesNotConnectOrStartRealtimeState) {
    RokaeHardware left, right;
    ASSERT_EQ(left.init(parameters("left")), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(right.init(parameters("right", true)), hardware_interface::CallbackReturn::SUCCESS);
    for (auto* hardware : {&left, &right}) {
        EXPECT_FALSE(RokaeHardwareTestPeer::connected(*hardware));
        EXPECT_FALSE(RokaeHardwareTestPeer::receiving(*hardware));
        EXPECT_FALSE(RokaeHardwareTestPeer::stateJoinable(*hardware));
        EXPECT_FALSE(RokaeHardwareTestPeer::active(*hardware));
        EXPECT_EQ(hardware->write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.002)),
                  hardware_interface::return_type::OK);
        RokaeHardwareTestPeer::start(*hardware);
        EXPECT_FALSE(RokaeHardwareTestPeer::running(*hardware));
        EXPECT_FALSE(RokaeHardwareTestPeer::joinable(*hardware));
    }
}

TEST_F(RokaeStartup, ModeCanBeConfiguredBeforeActivationWithoutConnecting) {
    RokaeHardware hardware;
    auto p = parameters("mode");
    p.hardware_info.hardware_parameters["ctrl_mode"] = "JOINT_IMPEDANCE";
    ASSERT_EQ(hardware.init(p), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_TRUE(RokaeHardwareTestPeer::changeMode(hardware).successful);
    EXPECT_FALSE(RokaeHardwareTestPeer::connected(hardware));
    EXPECT_FALSE(RokaeHardwareTestPeer::active(hardware));
}

TEST_F(RokaeStartup, FailedRightActivationRollsBackLoadedAndActiveSiblings) {
    RokaeHardware left, right, independent;
    ASSERT_EQ(left.init(parameters("left")), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(right.init(parameters("right", true)), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(independent.init(parameters("independent", false, "other_group")), hardware_interface::CallbackReturn::SUCCESS);
    RokaeHardwareTestPeer::activateForTest(left);
    RokaeHardwareTestPeer::activateForTest(independent);
    RokaeHardwareTestPeer::rollback(right);
    for (auto* hardware : {&left, &right}) {
        EXPECT_FALSE(RokaeHardwareTestPeer::active(*hardware));
        EXPECT_TRUE(RokaeHardwareTestPeer::faulted(*hardware));
        EXPECT_EQ(hardware->write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.002)),
                  hardware_interface::return_type::ERROR);
    }
    EXPECT_TRUE(RokaeHardwareTestPeer::active(independent));
    EXPECT_FALSE(RokaeHardwareTestPeer::faulted(independent));
}

TEST_F(RokaeStartup, ShutdownAndErrorJoinFinishedStateWorkerAndPreventRestart) {
    RokaeHardware hardware;
    ASSERT_EQ(hardware.init(parameters("shutdown")), hardware_interface::CallbackReturn::SUCCESS);
    RokaeHardwareTestPeer::activateForTest(hardware);
    RokaeHardwareTestPeer::finishedStateWorker(hardware);
    EXPECT_EQ(hardware.on_shutdown(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_FALSE(RokaeHardwareTestPeer::stateJoinable(hardware));
    EXPECT_FALSE(RokaeHardwareTestPeer::active(hardware));
    EXPECT_EQ(hardware.on_error(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(hardware.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.002)), hardware_interface::return_type::ERROR);
    EXPECT_EQ(hardware.on_cleanup(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_TRUE(RokaeHardwareTestPeer::faulted(hardware));
}

TEST_F(RokaeStartup, DestroyedSiblingIsRemovedBeforeAnotherActivationFails) {
    RokaeHardware right;
    ASSERT_EQ(right.init(parameters("right", true)), hardware_interface::CallbackReturn::SUCCESS);
    {
        RokaeHardware left;
        ASSERT_EQ(left.init(parameters("left")), hardware_interface::CallbackReturn::SUCCESS);
    }
    RokaeHardwareTestPeer::rollback(right);
    EXPECT_TRUE(RokaeHardwareTestPeer::faulted(right));
}
} // namespace rokae_ros2_control
