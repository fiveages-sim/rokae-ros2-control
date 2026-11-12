#include "rokae_ros2_control/rokae_hardware.h"
#include "rokae_ros2_control/ag2f90d_gripper.h"
#include "rokae_ros2_control/changingtek_model.h"
#include "rokae_ros2_control/jodell_gripper.h"
#include "rokae_ros2_control/linkerhand.h"

#include <gtest/gtest.h>
#include <future>

namespace rokae_ros2_control {

// Exercise the fault path without creating a robot connection or issuing SDK commands.
class RokaeHardwareTestPeer {
public:
    static Ag2f90dGripper* changingtekTool(RokaeHardware& hardware) {
        return dynamic_cast<Ag2f90dGripper*>(hardware.gripper_.get());
    }
    static bool connected(const RokaeHardware& hardware) { return hardware.arm_robot_ != nullptr; }
    static bool receiving(const RokaeHardware& hardware) { return hardware.state_receive_started_; }
    static bool active(const RokaeHardware& hardware) { return hardware.hardware_active_; }
    static bool faulted(const RokaeHardware& hardware) { return hardware.rt_control_faulted_; }
    static void activateForTest(RokaeHardware& hardware) { hardware.hardware_active_ = true; }
    static void finishedControlWorker(RokaeHardware& hardware) {
        hardware.rt_control_thread_ = std::thread([] {});
        hardware.rt_control_running_ = false;
    }
    using Snapshot = RokaeHardware::StateSnapshot;
    static bool publish(RokaeHardware& hardware, Snapshot snapshot) {
        return hardware.publishStateSnapshot(snapshot);
    }
    static void ageState(RokaeHardware& hardware, std::chrono::milliseconds age) {
        std::lock_guard<std::mutex> lock(hardware.state_mutex_);
        hardware.state_snapshot_.updated_at = std::chrono::steady_clock::now() - age;
    }
    static Snapshot snapshot(RokaeHardware& hardware) { return hardware.stateSnapshot(); }
    static const auto& positions(const RokaeHardware& hardware) { return hardware.joint_positions_; }
    static const auto& velocities(const RokaeHardware& hardware) { return hardware.joint_velocities_; }
    static const auto& efforts(const RokaeHardware& hardware) { return hardware.joint_efforts_; }
    static const auto& commands(const RokaeHardware& hardware) { return hardware.joint_position_commands_; }
    static const auto& targets(const RokaeHardware& hardware) { return hardware.target_joint_position_commands_; }
    static const auto& sent(const RokaeHardware& hardware) { return hardware.last_sent_joint_positions_; }
    static const auto& filters(const RokaeHardware& hardware) { return hardware.command_filters_; }
    static void rollback(RokaeHardware& hardware) { hardware.rollbackStartupGroup(); }
    static void installTool(RokaeHardware& hardware, std::unique_ptr<RokaeGripperInterface> tool) {
        hardware.gripper_ = std::move(tool);
    }
    static void fault(RokaeHardware& hardware) { hardware.latchRealtimeFault(); }
    static void start(RokaeHardware& hardware) { hardware.startRealtimeControl(); }
    static void stop(RokaeHardware& hardware) { hardware.stopRealtimeControl(); }
    static bool running(const RokaeHardware& hardware) { return hardware.rt_control_running_; }
    static void setRunning(RokaeHardware& hardware, bool running) { hardware.rt_control_running_ = running; }
    static const std::array<double, 7>& impedance(const RokaeHardware& hardware) { return hardware.joint_impedance_; }
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
        EXPECT_FALSE(RokaeHardwareTestPeer::snapshot(*hardware).valid);
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

TEST_F(RokaeStartup, ImpedanceModeRejectsStiffnessUpdatesBeforeStoppingRealtimeMove) {
    for (const bool running : {false, true}) {
        RokaeHardware hardware;
        auto p = parameters("stiffness_locked");
        p.hardware_info.hardware_parameters["ctrl_mode"] = "JOINT_IMPEDANCE";
        ASSERT_EQ(hardware.init(p), hardware_interface::CallbackReturn::SUCCESS);
        RokaeHardwareTestPeer::setRunning(hardware, running);
        const auto previous = RokaeHardwareTestPeer::impedance(hardware);
        const auto previous_parameter = hardware.get_node()->get_parameter("joint_impedance").as_double_array();
        const std::vector<double> stiffness(7, 100.0);
        // Even a batch switching to POSITION must not bypass the current-mode guard.
        for (const auto& batch : {
                std::vector<rclcpp::Parameter>{rclcpp::Parameter("joint_impedance", stiffness)},
                std::vector<rclcpp::Parameter>{rclcpp::Parameter("ctrl_mode", "POSITION"),
                                               rclcpp::Parameter("joint_impedance", stiffness)},
                std::vector<rclcpp::Parameter>{rclcpp::Parameter("joint_impedance", stiffness),
                                               rclcpp::Parameter("ctrl_mode", "POSITION")}}) {
            const auto result = hardware.get_node()->set_parameters_atomically(batch);
            EXPECT_FALSE(result.successful);
            EXPECT_NE(result.reason.find("switch to POSITION first"), std::string::npos);
            EXPECT_EQ(RokaeHardwareTestPeer::impedance(hardware), previous);
            EXPECT_EQ(hardware.get_node()->get_parameter("joint_impedance").as_double_array(), previous_parameter);
            EXPECT_EQ(hardware.get_node()->get_parameter("ctrl_mode").as_string(), "JOINT_IMPEDANCE");
            EXPECT_EQ(RokaeHardwareTestPeer::running(hardware), running);
            EXPECT_FALSE(RokaeHardwareTestPeer::restartRequested(hardware));
            EXPECT_FALSE(RokaeHardwareTestPeer::faulted(hardware));
        }
        RokaeHardwareTestPeer::setRunning(hardware, false);
    }
}

TEST_F(RokaeStartup, StiffnessCanBeChangedAfterSwitchingToPosition) {
    RokaeHardware hardware;
    auto p = parameters("stiffness_configurable");
    p.hardware_info.hardware_parameters["ctrl_mode"] = "JOINT_IMPEDANCE";
    ASSERT_EQ(hardware.init(p), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_TRUE(hardware.get_node()->set_parameters_atomically({
        rclcpp::Parameter("ctrl_mode", "POSITION")}).successful);
    const std::vector<double> stiffness(7, 100.0);
    ASSERT_TRUE(hardware.get_node()->set_parameters_atomically({
        rclcpp::Parameter("joint_impedance", stiffness)}).successful);
    EXPECT_EQ(hardware.get_node()->get_parameter("joint_impedance").as_double_array(), stiffness);
    for (const double value : RokaeHardwareTestPeer::impedance(hardware)) EXPECT_DOUBLE_EQ(value, 100.0);
    EXPECT_FALSE(RokaeHardwareTestPeer::restartRequested(hardware));
    ASSERT_TRUE(hardware.get_node()->set_parameters_atomically({
        rclcpp::Parameter("ctrl_mode", "JOINT_IMPEDANCE")}).successful);
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

TEST_F(RokaeStartup, ShutdownAndErrorJoinFinishedControlWorkerAndPreventRestart) {
    RokaeHardware hardware;
    ASSERT_EQ(hardware.init(parameters("shutdown")), hardware_interface::CallbackReturn::SUCCESS);
    RokaeHardwareTestPeer::activateForTest(hardware);
    RokaeHardwareTestPeer::Snapshot state;
    ASSERT_TRUE(RokaeHardwareTestPeer::publish(hardware, state));
    RokaeHardwareTestPeer::finishedControlWorker(hardware);
    EXPECT_EQ(hardware.on_shutdown(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_FALSE(RokaeHardwareTestPeer::joinable(hardware));
    EXPECT_FALSE(RokaeHardwareTestPeer::snapshot(hardware).valid);
    EXPECT_FALSE(RokaeHardwareTestPeer::active(hardware));
    EXPECT_EQ(hardware.on_error(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(hardware.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.002)), hardware_interface::return_type::ERROR);
    EXPECT_EQ(hardware.on_cleanup(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_TRUE(RokaeHardwareTestPeer::faulted(hardware));
}

TEST_F(RokaeStartup, CachedFeedbackIsAvailableBeforeRealtimeStartsWithoutSdkCalls) {
    RokaeHardware hardware;
    ASSERT_EQ(hardware.init(parameters("feedback")), hardware_interface::CallbackReturn::SUCCESS);
    RokaeHardwareTestPeer::Snapshot state;
    for (size_t i = 0; i < 7; ++i) {
        state.positions[i] = 0.01 * i;
        state.velocities[i] = 0.02 * i;
        state.efforts[i] = 0.03 * i;
    }
    ASSERT_TRUE(RokaeHardwareTestPeer::publish(hardware, state));
    ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
              hardware_interface::return_type::OK);
    for (size_t i = 0; i < 7; ++i) {
        EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::positions(hardware)[i], state.positions[i]);
        EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::velocities(hardware)[i], state.velocities[i]);
        EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::efforts(hardware)[i], state.efforts[i]);
    }
    EXPECT_FALSE(RokaeHardwareTestPeer::connected(hardware));
    EXPECT_FALSE(RokaeHardwareTestPeer::running(hardware));
}

TEST_F(RokaeStartup, NonFiniteFeedbackCannotReplaceTheLastCompleteSnapshot) {
    RokaeHardware hardware;
    ASSERT_EQ(hardware.init(parameters("invalid_feedback")), hardware_interface::CallbackReturn::SUCCESS);
    RokaeHardwareTestPeer::Snapshot state;
    state.positions.fill(0.1);
    ASSERT_TRUE(RokaeHardwareTestPeer::publish(hardware, state));
    for (int field = 0; field < 3; ++field) {
        auto bad = state;
        auto& values = field == 0 ? bad.positions : field == 1 ? bad.velocities : bad.efforts;
        values[3] = std::numeric_limits<double>::quiet_NaN();
        EXPECT_FALSE(RokaeHardwareTestPeer::publish(hardware, bad));
        const auto cached = RokaeHardwareTestPeer::snapshot(hardware);
        EXPECT_EQ(cached.positions, state.positions);
        EXPECT_EQ(cached.velocities, state.velocities);
        EXPECT_EQ(cached.efforts, state.efforts);
    }
}

TEST_F(RokaeStartup, ConcurrentFeedbackAndReadKeepPositionVelocityAndEffortCoherent) {
    RokaeHardware hardware;
    ASSERT_EQ(hardware.init(parameters("coherent_feedback")), hardware_interface::CallbackReturn::SUCCESS);
    std::atomic<bool> done{false};
    std::thread producer([&] {
        for (int frame = 1; frame <= 10000; ++frame) {
            RokaeHardwareTestPeer::Snapshot state;
            state.positions.fill(frame);
            state.velocities.fill(frame * 2);
            state.efforts.fill(frame * 3);
            RokaeHardwareTestPeer::publish(hardware, state);
        }
        done = true;
    });
    do {
        EXPECT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                  hardware_interface::return_type::OK);
        const double frame = RokaeHardwareTestPeer::positions(hardware)[0];
        for (size_t i = 0; i < 7; ++i) {
            EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::positions(hardware)[i], frame);
            EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::velocities(hardware)[i], frame * 2);
            EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::efforts(hardware)[i], frame * 3);
        }
    } while (!done);
    producer.join();
}

TEST_F(RokaeStartup, PositionModeUsesFreshSnapshotToSyncCommandsAndFiltersWithoutSdkConnection) {
    RokaeHardware hardware;
    auto p = parameters("position_sync");
    p.hardware_info.hardware_parameters["ctrl_mode"] = "JOINT_IMPEDANCE";
    ASSERT_EQ(hardware.init(p), hardware_interface::CallbackReturn::SUCCESS);
    RokaeHardwareTestPeer::Snapshot state;
    state.positions.fill(0.15);
    state.velocities.fill(0.05);
    ASSERT_TRUE(RokaeHardwareTestPeer::publish(hardware, state));
    RokaeHardwareTestPeer::activateForTest(hardware);
    const auto change = hardware.get_node()->set_parameters_atomically({
        rclcpp::Parameter("ctrl_mode", "POSITION")});
    ASSERT_TRUE(change.successful) << change.reason;
    for (size_t i = 0; i < 7; ++i) {
        EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::commands(hardware)[i], state.positions[i]);
        EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::targets(hardware)[i], state.positions[i]);
        EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::sent(hardware)[i], state.positions[i]);
        EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::filters(hardware)[i].q, state.positions[i]);
        EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::filters(hardware)[i].qd, state.velocities[i]);
    }
    EXPECT_FALSE(RokaeHardwareTestPeer::connected(hardware));
}

TEST_F(RokaeStartup, StaleOrMissingStateRejectsPositionSwitchWithoutChangingCommands) {
    for (const bool stale : {false, true}) {
        RokaeHardware hardware;
        auto p = parameters("stale_sync");
        p.hardware_info.hardware_parameters["ctrl_mode"] = "JOINT_IMPEDANCE";
        ASSERT_EQ(hardware.init(p), hardware_interface::CallbackReturn::SUCCESS);
        if (stale) {
            RokaeHardwareTestPeer::Snapshot state;
            state.positions.fill(0.15);
            ASSERT_TRUE(RokaeHardwareTestPeer::publish(hardware, state));
            RokaeHardwareTestPeer::ageState(hardware, std::chrono::seconds(1));
        }
        RokaeHardwareTestPeer::activateForTest(hardware);
        const auto previous = RokaeHardwareTestPeer::commands(hardware);
        const auto result = hardware.get_node()->set_parameters_atomically({
            rclcpp::Parameter("ctrl_mode", "POSITION")});
        EXPECT_FALSE(result.successful);
        EXPECT_EQ(hardware.get_node()->get_parameter("ctrl_mode").as_string(), "JOINT_IMPEDANCE");
        EXPECT_EQ(RokaeHardwareTestPeer::commands(hardware), previous);
        EXPECT_FALSE(RokaeHardwareTestPeer::connected(hardware));
    }
}

TEST_F(RokaeStartup, FailedPositionSyncAfterMoveStopCannotResumeOldCommands) {
    RokaeHardware hardware;
    auto p = parameters("failed_running_sync");
    p.hardware_info.hardware_parameters["ctrl_mode"] = "JOINT_IMPEDANCE";
    ASSERT_EQ(hardware.init(p), hardware_interface::CallbackReturn::SUCCESS);
    RokaeHardwareTestPeer::activateForTest(hardware);
    RokaeHardwareTestPeer::setRunning(hardware, true);
    const auto result = hardware.get_node()->set_parameters_atomically({
        rclcpp::Parameter("ctrl_mode", "POSITION")});
    EXPECT_FALSE(result.successful);
    EXPECT_TRUE(RokaeHardwareTestPeer::faulted(hardware));
    EXPECT_FALSE(RokaeHardwareTestPeer::running(hardware));
    EXPECT_FALSE(RokaeHardwareTestPeer::restartRequested(hardware));
    EXPECT_EQ(hardware.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
              hardware_interface::return_type::ERROR);
}

class ToolDynamicsProbe : public RokaeGripperInterface {
public:
    void stop() override {}
    bool communicationStopped() const override { return true; }
    double torque = 1.0, velocity = 1.0, storage = 0.0;
    int updates = 0;
    std::string name = "gripper_joint";
    bool initialize(rokae::xMateErProRobot&) override { return false; }
    bool readStatusPeriodic() override { return false; }
    bool writeCommandPeriodic() override { return false; }
    bool hasGripper() const override { return true; }
    const std::string& getJointName() const override { return name; }
    bool isInitialized() const override { return false; }
    double* getPositionPtr() override { return &storage; }
    double* getVelocityPtr() override { return &storage; }
    double* getEffortPtr() override { return &storage; }
    double* getPositionCommandPtr() override { return &storage; }
    void setToolDynamics(double t, double v) override { torque = t; velocity = v; ++updates; }
    void exportStateInterfaces(std::vector<hardware_interface::StateInterface::ConstSharedPtr>&) override {}
    void exportCommandInterfaces(std::vector<hardware_interface::CommandInterface::SharedPtr>&) override {}
};

class ShutdownToolProbe : public ToolDynamicsProbe {
public:
    explicit ShutdownToolProbe(std::function<void()> action, bool stopped = true)
        : action_(std::move(action)), stopped_(stopped) {}
    void stop() override { action_(); }
    bool communicationStopped() const override { return stopped_; }
private:
    std::function<void()> action_;
    bool stopped_;
};

TEST_F(RokaeStartup, UnresolvedToolWorkerCannotReturnIntoSdkDestructors) {
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    EXPECT_EXIT({
        RokaeHardware hardware;
        if (hardware.init(parameters("unresolved")) != hardware_interface::CallbackReturn::SUCCESS) std::_Exit(2);
        RokaeHardwareTestPeer::installTool(hardware, std::make_unique<ShutdownToolProbe>([] {
            throw std::runtime_error("unresolved communication worker");
        }, false));
        hardware.on_deactivate(rclcpp_lifecycle::State{});
        std::_Exit(3);
    }, ::testing::KilledBySignal(SIGKILL), "terminating controller process");
}

TEST_F(RokaeStartup, DeactivationCancelsAndJoinsBothArmsBeforeStoppingAnyTool) {
    RokaeHardware left, right, independent;
    ASSERT_EQ(left.init(parameters("left")), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(right.init(parameters("right")), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(independent.init(parameters("other", false, "other_group")), hardware_interface::CallbackReturn::SUCCESS);
    for (auto* arm : {&left, &right, &independent}) {
        RokaeHardwareTestPeer::activateForTest(*arm);
        RokaeHardwareTestPeer::Snapshot state;
        ASSERT_TRUE(RokaeHardwareTestPeer::publish(*arm, state));
        RokaeHardwareTestPeer::finishedControlWorker(*arm);
    }
    std::vector<std::string> stopped;
    for (auto* arm : {&left, &right}) {
        const std::string side = arm == &left ? "left" : "right";
        RokaeHardwareTestPeer::installTool(*arm, std::make_unique<ShutdownToolProbe>([&, side] {
            for (auto* peer : {&left, &right}) {
                EXPECT_FALSE(RokaeHardwareTestPeer::active(*peer));
                EXPECT_FALSE(RokaeHardwareTestPeer::running(*peer));
                EXPECT_FALSE(RokaeHardwareTestPeer::joinable(*peer));
                EXPECT_FALSE(RokaeHardwareTestPeer::snapshot(*peer).valid);
            }
            stopped.push_back(side);
        }));
    }
    EXPECT_EQ(right.on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(stopped, (std::vector<std::string>{"left", "right"}));
    EXPECT_TRUE(RokaeHardwareTestPeer::active(independent));
    EXPECT_TRUE(RokaeHardwareTestPeer::snapshot(independent).valid);
    EXPECT_TRUE(RokaeHardwareTestPeer::joinable(independent));
    // The manager will subsequently invoke the remaining lifecycle callbacks.
    EXPECT_EQ(left.on_shutdown(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(right.on_cleanup(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(stopped.size(), 6u);
    for (auto* arm : {&left, &right}) RokaeHardwareTestPeer::installTool(*arm, nullptr);
}

TEST_F(RokaeStartup, ToolStopFailureStillStopsSiblingAndCanBeRetried) {
    RokaeHardware left, right;
    ASSERT_EQ(left.init(parameters("left")), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(right.init(parameters("right")), hardware_interface::CallbackReturn::SUCCESS);
    bool fail = true;
    int left_stops = 0, right_stops = 0;
    RokaeHardwareTestPeer::installTool(left, std::make_unique<ShutdownToolProbe>([&] {
        ++left_stops;
        if (fail) throw std::runtime_error("simulated tool stop failure");
    }));
    RokaeHardwareTestPeer::installTool(right, std::make_unique<ShutdownToolProbe>([&] { ++right_stops; }));
    RokaeHardwareTestPeer::activateForTest(left);
    RokaeHardwareTestPeer::activateForTest(right);
    EXPECT_EQ(right.on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::ERROR);
    EXPECT_EQ(left_stops, 1);
    EXPECT_EQ(right_stops, 1);
    EXPECT_FALSE(RokaeHardwareTestPeer::active(left));
    EXPECT_FALSE(RokaeHardwareTestPeer::active(right));
    fail = false;
    EXPECT_EQ(left.on_cleanup(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(left_stops, 2);
    EXPECT_EQ(right_stops, 2);
    for (auto* arm : {&left, &right}) RokaeHardwareTestPeer::installTool(*arm, nullptr);
}

TEST_F(RokaeStartup, ParameterCallbacksWaitUntilGroupShutdownFinishes) {
    RokaeHardware left, right;
    ASSERT_EQ(left.init(parameters("left")), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(right.init(parameters("right")), hardware_interface::CallbackReturn::SUCCESS);
    std::promise<void> stopping_tool, release_tool, parameter_started;
    auto stopping_future = stopping_tool.get_future();
    auto release_future = release_tool.get_future();
    auto parameter_started_future = parameter_started.get_future();
    RokaeHardwareTestPeer::installTool(left, std::make_unique<ShutdownToolProbe>([&] {
        stopping_tool.set_value();
        release_future.wait();
    }));
    auto shutdown = std::async(std::launch::async, [&] {
        return right.on_deactivate(rclcpp_lifecycle::State{});
    });
    stopping_future.wait();
    auto parameter = std::async(std::launch::async, [&] {
        parameter_started.set_value();
        return right.get_node()->set_parameter(rclcpp::Parameter("ctrl_mode", "JOINT_IMPEDANCE"));
    });
    parameter_started_future.wait();
    const auto during_shutdown = parameter.wait_for(std::chrono::milliseconds(50));
    release_tool.set_value();
    EXPECT_EQ(shutdown.get(), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_TRUE(parameter.get().successful);
    EXPECT_EQ(during_shutdown, std::future_status::timeout);
    RokaeHardwareTestPeer::installTool(left, nullptr);
}

TEST_F(RokaeStartup, DeclaresInstanceLocalToolParametersAndReadsHardwareDefaults) {
    RokaeHardware left, right;
    auto p = parameters("left");
    p.hardware_info.hardware_parameters["tool_torque"] = "0.3";
    p.hardware_info.hardware_parameters["tool_velocity"] = "0.4";
    ASSERT_EQ(left.init(p), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(right.init(parameters("right", true)), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_DOUBLE_EQ(left.get_node()->get_parameter("tool_torque").as_double(), 0.3);
    EXPECT_DOUBLE_EQ(left.get_node()->get_parameter("tool_velocity").as_double(), 0.4);
    EXPECT_DOUBLE_EQ(right.get_node()->get_parameter("tool_torque").as_double(), 1.0);
    EXPECT_DOUBLE_EQ(right.get_node()->get_parameter("tool_velocity").as_double(), 1.0);
    for (auto* hardware : {&left, &right}) {
        for (const auto* old : {"left_tool_torque", "left_tool_velocity", "right_tool_torque", "right_tool_velocity"}) {
            EXPECT_FALSE(hardware->get_node()->has_parameter(old));
        }
    }
}

TEST_F(RokaeStartup, ToolParameterUpdatesAffectOnlyOwningInstanceAndKeepOtherValue) {
    RokaeHardware left, right;
    ASSERT_EQ(left.init(parameters("left")), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(right.init(parameters("right", true)), hardware_interface::CallbackReturn::SUCCESS);
    auto left_probe = std::make_unique<ToolDynamicsProbe>();
    auto right_probe = std::make_unique<ToolDynamicsProbe>();
    auto* l = left_probe.get();
    auto* r = right_probe.get();
    RokaeHardwareTestPeer::installTool(left, std::move(left_probe));
    RokaeHardwareTestPeer::installTool(right, std::move(right_probe));
    ASSERT_TRUE(right.get_node()->set_parameters_atomically({
        rclcpp::Parameter("tool_torque", 0.3), rclcpp::Parameter("tool_velocity", 0.4)}).successful);
    EXPECT_DOUBLE_EQ(r->torque, 0.3);
    EXPECT_DOUBLE_EQ(r->velocity, 0.4);
    EXPECT_EQ(l->updates, 0);
    ASSERT_TRUE(right.get_node()->set_parameter(rclcpp::Parameter("tool_velocity", 1)).successful);
    EXPECT_DOUBLE_EQ(r->torque, 0.3);
    EXPECT_DOUBLE_EQ(r->velocity, 1.0);
    EXPECT_EQ(l->updates, 0);
    for (double invalid : {-0.1, 1.1, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        EXPECT_FALSE(right.get_node()->set_parameters_atomically({
            rclcpp::Parameter("tool_torque", 0.8), rclcpp::Parameter("tool_velocity", invalid)}).successful);
        EXPECT_DOUBLE_EQ(r->torque, 0.3);
        EXPECT_DOUBLE_EQ(r->velocity, 1.0);
    }
    EXPECT_FALSE(right.get_node()->set_parameter(rclcpp::Parameter("tool_torque", "bad")).successful);
}

class ChangingtekGripperTestPeer {
public:
    static const Ag2f90dGripper::Config& config(const Ag2f90dGripper& g) { return g.config_; }
    static int torque(const Ag2f90dGripper& g) { return g.command_torque_; }
    static int velocity(const Ag2f90dGripper& g) { return g.command_velocity_; }
    static int encode(const Ag2f90dGripper& g, double p) { return g.normalizedToRegister(p); }
    static double decode(const Ag2f90dGripper& g, int p) { return g.registerToNormalized(p); }
    static bool worker(const Ag2f90dGripper& g) { return g.command_thread_.joinable(); }
    static void finishedWorker(Ag2f90dGripper& g) {
        g.command_thread_ = std::thread([] {});
        g.command_queue_.push({123});
        g.initialized_ = true;
    }
    static bool emptyQueue(const Ag2f90dGripper& g) { return g.command_queue_.empty(); }
};

TEST(ChangingtekModels, PreservesAliasesAndRejectsUnknownTypes) {
    for (const auto* alias : {"ag2f90_c", "ag2f90c", "ag2f90", "changingtek90c", "AG2F90_C"}) {
        const auto m = changingtek::modelFor(alias);
        ASSERT_TRUE(m);
        EXPECT_STREQ(m->name, "AG2F90C");
    }
    for (const auto* alias : {"ag2f90_d", "ag2f90d", "changingtek90d"}) {
        const auto m = changingtek::modelFor(alias);
        ASSERT_TRUE(m);
        EXPECT_STREQ(m->name, "AG2F90D");
    }
    EXPECT_FALSE(changingtek::modelFor("unknown"));
    EXPECT_FALSE(changingtek::modelFor(""));
}

TEST_F(RokaeStartup, ChangingtekFactorySelectsAllFourModelRanges) {
    struct Expected { const char* type; int close; int feedback; int dynamics; };
    for (const auto& expected : {Expected{"ag2f90_c", 9000, 0x060D, 255},
                               Expected{"ag2f90_d", 9000, 0x0418, 255},
                               Expected{"AG2F120S", 12000, 0x060D, 100},
                               Expected{"ag2f120s_d", 1000, 0x0418, 100}}) {
        for (const auto* side : {"left", "right"}) {
            RokaeHardware hardware;
            auto p = parameters(side);
            p.hardware_info.hardware_parameters["gripper_type"] = expected.type;
            p.hardware_info.hardware_parameters["tool_torque"] = "0.5";
            p.hardware_info.hardware_parameters["tool_velocity"] = "0.5";
            hardware_interface::ComponentInfo joint;
            joint.name = std::string(side) + "_gripper_joint";
            p.hardware_info.joints.push_back(joint);
            ASSERT_EQ(hardware.init(p), hardware_interface::CallbackReturn::SUCCESS);
            auto* tool = RokaeHardwareTestPeer::changingtekTool(hardware);
            ASSERT_NE(tool, nullptr);
            const auto& config = ChangingtekGripperTestPeer::config(*tool);
            EXPECT_EQ(config.close_pos, expected.close);
            EXPECT_EQ(config.feedback_reg, expected.feedback);
            EXPECT_EQ(config.max_torque, expected.dynamics);
            EXPECT_EQ(config.max_velocity, expected.dynamics);
            EXPECT_EQ(ChangingtekGripperTestPeer::torque(*tool), static_cast<int>(expected.dynamics * 0.5));
            EXPECT_EQ(ChangingtekGripperTestPeer::velocity(*tool), std::lround(expected.dynamics * 0.5));
            EXPECT_EQ(ChangingtekGripperTestPeer::encode(*tool, 0.0), expected.close);
            EXPECT_EQ(ChangingtekGripperTestPeer::encode(*tool, 1.0), 0);
            EXPECT_EQ(ChangingtekGripperTestPeer::encode(*tool, 0.5), expected.close / 2);
            EXPECT_DOUBLE_EQ(ChangingtekGripperTestPeer::decode(*tool, 0), 1.0);
            EXPECT_DOUBLE_EQ(ChangingtekGripperTestPeer::decode(*tool, expected.close), 0.0);
            ASSERT_TRUE(hardware.get_node()->set_parameters_atomically({
                rclcpp::Parameter("tool_torque", 1.0), rclcpp::Parameter("tool_velocity", 1.0)}).successful);
            EXPECT_EQ(ChangingtekGripperTestPeer::torque(*tool), expected.dynamics);
            EXPECT_EQ(ChangingtekGripperTestPeer::velocity(*tool), expected.dynamics);
            EXPECT_FALSE(ChangingtekGripperTestPeer::worker(*tool));
            EXPECT_FALSE(RokaeHardwareTestPeer::connected(hardware));
        }
    }
}

TEST_F(RokaeStartup, ChangingtekModelAllowsRegisterOverrides) {
    RokaeHardware hardware;
    auto p = parameters("override");
    p.hardware_info.hardware_parameters["gripper_type"] = "ag2f120s";
    p.hardware_info.hardware_parameters["gripper_close_pos"] = "11000";
    p.hardware_info.hardware_parameters["gripper_feedback_reg"] = "1500";
    hardware_interface::ComponentInfo joint;
    joint.name = "right_gripper_joint";
    p.hardware_info.joints.push_back(joint);
    ASSERT_EQ(hardware.init(p), hardware_interface::CallbackReturn::SUCCESS);
    auto* tool = RokaeHardwareTestPeer::changingtekTool(hardware);
    ASSERT_NE(tool, nullptr);
    EXPECT_EQ(ChangingtekGripperTestPeer::config(*tool).close_pos, 11000);
    EXPECT_EQ(ChangingtekGripperTestPeer::config(*tool).feedback_reg, 1500);
    EXPECT_EQ(ChangingtekGripperTestPeer::config(*tool).max_torque, 100);
}

TEST(ChangingtekLifecycle, StopJoinsWorkerAndClearsPendingCommands) {
    Ag2f90dGripper tool(rclcpp::get_logger("test"), std::make_shared<rclcpp::Clock>(), "gripper_joint", {});
    EXPECT_FALSE(ChangingtekGripperTestPeer::worker(tool));
    ChangingtekGripperTestPeer::finishedWorker(tool);
    tool.stop();
    EXPECT_FALSE(tool.isInitialized());
    EXPECT_FALSE(ChangingtekGripperTestPeer::worker(tool));
    EXPECT_TRUE(ChangingtekGripperTestPeer::emptyQueue(tool));
    tool.stop();
}

// Model an SDK transaction already in flight, without a robot connection. stop()
// must wait for it before the hardware owner is allowed to release the SDK.
class Rs485LifecycleTestPeer {
public:
    static void start(JodellGripper& tool, std::function<void()> transaction) {
        tool.command_queue_.push({123, 100, 100});
        tool.initialized_ = true;
        tool.thread_running_ = true;
        tool.background_status_valid_ = true;
        tool.command_thread_ = std::thread(std::move(transaction));
    }
    static void start(LinkerHand& tool, std::function<void()> transaction) {
        tool.initialized_ = true;
        tool.running_ = true;
        tool.command_pending_ = true;
        tool.have_valid_positions_ = true;
        tool.communication_thread_ = std::thread(std::move(transaction));
    }
    static bool worker(const JodellGripper& tool) { return tool.command_thread_.joinable(); }
    static bool worker(const LinkerHand& tool) { return tool.communication_thread_.joinable(); }
    static bool cleared(const JodellGripper& tool) {
        return !tool.thread_running_ && tool.robot_ptr_ == nullptr &&
               tool.command_queue_.empty() && !tool.background_status_valid_;
    }
    static bool cleared(const LinkerHand& tool) {
        return !tool.running_ && tool.robot_ == nullptr &&
               !tool.command_pending_ && !tool.have_valid_positions_;
    }
};

template<class Tool>
void expectStopWaitsForTransaction(Tool& tool) {
    EXPECT_FALSE(Rs485LifecycleTestPeer::worker(tool));
    std::promise<void> entered, release;
    auto entered_future = entered.get_future();
    auto release_future = release.get_future();
    std::atomic<bool> exited{false};
    Rs485LifecycleTestPeer::start(tool, [&] {
        entered.set_value();
        release_future.wait();
        exited = true;
    });
    entered_future.wait();
    RokaeGripperInterface& interface = tool;
    std::promise<void> stopping;
    auto stopping_future = stopping.get_future();
    auto stopped = std::async(std::launch::async, [&] {
        stopping.set_value();
        interface.stop();
    });
    stopping_future.wait();
    const auto before_release = stopped.wait_for(std::chrono::milliseconds(50));
    // Release even if stop() incorrectly returned early, so the test can clean up.
    release.set_value();
    stopped.get();
    EXPECT_EQ(before_release, std::future_status::timeout);
    EXPECT_TRUE(exited);
    EXPECT_FALSE(tool.isInitialized());
    EXPECT_FALSE(tool.readStatusPeriodic());
    EXPECT_FALSE(tool.writeCommandPeriodic());
    EXPECT_FALSE(Rs485LifecycleTestPeer::worker(tool));
    EXPECT_TRUE(Rs485LifecycleTestPeer::cleared(tool));
    EXPECT_TRUE(tool.communicationStopped());
    interface.stop();
    EXPECT_TRUE(Rs485LifecycleTestPeer::cleared(tool));
}

TEST(Rs485Lifecycle, JodellStopWaitsForTransactionAndDiscardsPendingWork) {
    JodellGripper tool(rclcpp::get_logger("test"), std::make_shared<rclcpp::Clock>(), "gripper_joint");
    expectStopWaitsForTransaction(tool);
    // A subsequent activation can create a new worker after the first was joined.
    expectStopWaitsForTransaction(tool);
}

TEST(Rs485Lifecycle, AllLinkerHandModelsStopBeforeSdkRelease) {
    for (const auto* type : {"o6", "l6", "o7"}) {
        for (bool right : {false, true}) {
            SCOPED_TRACE(std::string(type) + (right ? " right" : " left"));
            LinkerHand::Config config;
            config.hand_type = type;
            config.right_hand = right;
            const auto model = linkerhand::modelFor(type, right);
            std::vector<std::string> names;
            for (const auto& suffix : model.suffixes) {
                names.push_back(std::string(right ? "right_hand_" : "left_hand_") + suffix);
            }
            LinkerHand tool(rclcpp::get_logger("test"), std::make_shared<rclcpp::Clock>(), names, config);
            expectStopWaitsForTransaction(tool);
            expectStopWaitsForTransaction(tool);
        }
    }
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
