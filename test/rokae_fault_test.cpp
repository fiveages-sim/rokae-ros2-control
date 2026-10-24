#include "rokae_ros2_control/rokae_hardware.h"
#include "rokae_ros2_control/ag2f90d_gripper.h"
#include "rokae_ros2_control/changingtek_model.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rokae_ros2_control {

// Exercise the fault path without creating a robot connection or issuing SDK commands.
class RokaeHardwareTestPeer {
public:
    static void prepareLimits(RokaeHardware& hardware, const std::string& xml) {
        hardware.arm_joint_count_ = 1;
        hardware.joint_names_ = {"joint"};
        hardware.configureArmPositionLimits(xml);
        hardware.command_filters_.resize(1);
        hardware.target_joint_position_commands_ = {0.0};
    }
    static double command(RokaeHardware& hardware, double target, bool filtered = false) {
        hardware.use_double_s_filter_ = filtered;
        hardware.target_joint_position_commands_[0] = target;
        return hardware.nextArmPositionCommand(0);
    }
    static void filterState(RokaeHardware& hardware, double q, double qd, double qdd) {
        hardware.command_filters_[0].reset(q, qd, qdd);
    }
    static double filterVelocity(const RokaeHardware& hardware) { return hardware.command_filters_[0].qd; }
    static double filterAcceleration(const RokaeHardware& hardware) { return hardware.command_filters_[0].qdd; }
    static void setArmCommands(RokaeHardware& hardware, double position) {
        std::fill(hardware.joint_position_commands_.begin(), hardware.joint_position_commands_.end(), position);
    }
    static void setFeedback(RokaeHardware& hardware, double position) {
        std::fill(hardware.joint_positions_.begin(), hardware.joint_positions_.end(), position);
    }
    static const std::vector<double>& feedback(const RokaeHardware& hardware) { return hardware.joint_positions_; }
    static const std::vector<double>& targets(const RokaeHardware& hardware) { return hardware.target_joint_position_commands_; }
    static Ag2f90dGripper* changingtekTool(RokaeHardware& hardware) {
        return dynamic_cast<Ag2f90dGripper*>(hardware.gripper_.get());
    }
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
        p.hardware_info.original_xml = "<robot name='test'><link name='base'/>";
        for (int i = 0; i < 7; ++i) {
            const auto link = "link" + std::to_string(i);
            const auto parent = i == 0 ? "base" : "link" + std::to_string(i - 1);
            p.hardware_info.original_xml += "<link name='" + link + "'/><joint name='" +
                p.hardware_info.joints[i].name + "' type='revolute'><parent link='" + parent +
                "'/><child link='" + link + "'/><limit lower='-1' upper='1' effort='10' velocity='2'/></joint>";
        }
        p.hardware_info.original_xml += "</robot>";
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

static const std::string kPositionLimitsUrdf =
    "<robot name='test'><link name='base'/><link name='tip'/>"
    "<joint name='joint' type='revolute'><parent link='base'/><child link='tip'/>"
    "<limit lower='-2' upper='2' effort='10' velocity='2'/>"
    "<safety_controller soft_lower_limit='-1' soft_upper_limit='1' k_velocity='1'/></joint></robot>";

TEST(RokaePositionLimits, UsesUrdfSoftLimitsAndAcceptsBoundaryCommands) {
    RokaeHardware hardware;
    RokaeHardwareTestPeer::prepareLimits(hardware, kPositionLimitsUrdf);
    for (const double target : {-10.0, -1.0, -0.5, 0.0, 0.5, 1.0, 10.0}) {
        EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::command(hardware, target), std::clamp(target, -1.0, 1.0));
        EXPECT_FALSE(RokaeHardwareTestPeer::faulted(hardware));
    }
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity()}) {
        EXPECT_THROW(RokaeHardwareTestPeer::command(hardware, invalid), std::runtime_error);
    }
}

TEST(RokaePositionLimits, FilterOvershootResetsStateAndCanMoveBackInward) {
    RokaeHardware hardware;
    RokaeHardwareTestPeer::prepareLimits(hardware, kPositionLimitsUrdf);
    for (const double sign : {-1.0, 1.0}) {
        RokaeHardwareTestPeer::filterState(hardware, sign, sign * 2.0, sign * 3.0);
        EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::command(hardware, sign * 10.0, true), sign);
        EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::filterVelocity(hardware), 0.0);
        EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::filterAcceleration(hardware), 0.0);
        double output = sign;
        for (int cycle = 0; cycle < 1000; ++cycle) {
            output = RokaeHardwareTestPeer::command(hardware, 0.0, true);
            ASSERT_TRUE(std::isfinite(output));
            ASSERT_GE(output, -1.0);
            ASSERT_LE(output, 1.0);
        }
        EXPECT_LT(std::abs(output), 1.0);
    }
}

TEST(RokaePositionLimits, FallsBackToUrdfJointLimitsWithoutSafetyController) {
    RokaeHardware hardware;
    auto xml = kPositionLimitsUrdf;
    const auto start = xml.find("<safety_controller");
    xml.erase(start, xml.find("/>", start) + 2 - start);
    RokaeHardwareTestPeer::prepareLimits(hardware, xml);
    EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::command(hardware, 10.0), 2.0);
    EXPECT_DOUBLE_EQ(RokaeHardwareTestPeer::command(hardware, -10.0), -2.0);
}

TEST(RokaePositionLimits, RejectsMissingAndInvertedLimits) {
    RokaeHardware hardware;
    auto xml = kPositionLimitsUrdf;
    const auto start = xml.find("<limit ");
    xml.erase(start, xml.find("/>", start) + 2 - start);
    EXPECT_THROW(RokaeHardwareTestPeer::prepareLimits(hardware, xml), std::runtime_error);
    xml = kPositionLimitsUrdf;
    const auto lower = xml.find("soft_lower_limit='-1'");
    xml.replace(lower, std::string("soft_lower_limit='-1'").size(), "soft_lower_limit='1.5'");
    EXPECT_THROW(RokaeHardwareTestPeer::prepareLimits(hardware, xml), std::runtime_error);
}

TEST_F(RokaeStartup, OutOfRangeCommandsAreClampedWithoutFaultOrChangingFeedback) {
    RokaeHardware hardware;
    ASSERT_EQ(hardware.init(parameters("arm")), hardware_interface::CallbackReturn::SUCCESS);
    RokaeHardwareTestPeer::activateForTest(hardware);
    RokaeHardwareTestPeer::setFeedback(hardware, 1.1);
    for (const double target : {-10.0, 10.0}) {
        RokaeHardwareTestPeer::setArmCommands(hardware, target);
        EXPECT_EQ(hardware.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.002)),
                  hardware_interface::return_type::OK);
        EXPECT_FALSE(RokaeHardwareTestPeer::faulted(hardware));
        for (const double position : RokaeHardwareTestPeer::targets(hardware)) {
            EXPECT_DOUBLE_EQ(position, target < 0.0 ? -1.0 : 1.0);
        }
        for (const double position : RokaeHardwareTestPeer::feedback(hardware)) EXPECT_DOUBLE_EQ(position, 1.1);
    }
    const auto previous = RokaeHardwareTestPeer::targets(hardware);
    RokaeHardwareTestPeer::setArmCommands(hardware, std::numeric_limits<double>::quiet_NaN());
    EXPECT_EQ(hardware.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.002)),
              hardware_interface::return_type::ERROR);
    EXPECT_TRUE(RokaeHardwareTestPeer::faulted(hardware));
    EXPECT_EQ(RokaeHardwareTestPeer::targets(hardware), previous);
}

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

class ToolDynamicsProbe : public RokaeGripperInterface {
public:
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
