#include "rokae_ros2_control/erg32_gripper.h"
#include <gtest/gtest.h>
#include <tuple>

namespace rokae_ros2_control {
class Erg32TestPeer {
public:
    static void transport(Erg32Gripper& g, std::function<bool(int, int, std::vector<int>&)> f) { g.transfer_ = std::move(f); }
    static bool initialize(Erg32Gripper& g) { return g.initializeDevice(); }
    static bool waitFault(Erg32Gripper& g) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (!g.faulted_ && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return g.faulted_;
    }
    static bool waitPosition(Erg32Gripper& g, double expected) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (std::chrono::steady_clock::now() < deadline) {
            {
                std::lock_guard<std::mutex> lock(g.mutex_);
                if (std::abs(g.feedback_.position[0] - expected) < 1e-6) return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return false;
    }
};

TEST(Erg32Protocol, AbsoluteEncodingAndLimits) {
    const auto p = erg32::encode({-erg32::pi / 2, 0.016}, 1.0, 0.5);
    EXPECT_EQ(p[0], 0x8000);
    EXPECT_EQ(p[1], 0xFF01);
    EXPECT_EQ(p[2], 0xFFA6);
    EXPECT_EQ(p[3], 0xFF80);
    EXPECT_EQ(p[4], 0);
    EXPECT_EQ(p[5], 1);
    EXPECT_EQ(erg32::encode({100, -1}, 0, 0)[2], 180);
    EXPECT_EQ(erg32::encode({100, -1}, 0, 0)[0], 255);
    EXPECT_THROW(erg32::encode({NAN, 0}, 1, 1), std::invalid_argument);
    EXPECT_THROW(erg32::encode({0, 0}, INFINITY, 1), std::invalid_argument);
}

TEST(Erg32Protocol, DecodesStatusFaultsAndSignedPosition) {
    std::array<uint16_t, 9> registers{0x007A, 0x0898, 0x8040, 0x1200, 0xFFA6, 0x3420, 1, 0, 0x2818};
    const auto f = erg32::decode(registers);
    EXPECT_TRUE(f.ready());
    EXPECT_FALSE(f.healthy());
    EXPECT_EQ(f.fault[0], 8);
    EXPECT_EQ(f.object[1], 3);
    EXPECT_TRUE(f.dropped);
    EXPECT_DOUBLE_EQ(f.position[0], -erg32::pi / 2);
    EXPECT_NEAR(f.position[1], (1 - 64.0 / 255) * 0.016, 1e-12);
    EXPECT_EQ(f.raw_current[1], 0x12);
    EXPECT_EQ(f.voltage, 24);
    EXPECT_EQ(f.temperature, 40);
    registers[6] = 0xFF01;
    EXPECT_EQ(erg32::decode(registers).turns, -1);
}

TEST(Erg32Protocol, IndependentAxisWritesAndTriggerAfterParameters) {
    auto last = erg32::encode({0, 0}, 0.5, 0.5);
    const auto rotated = erg32::encode({erg32::pi / 2, 0}, 0.5, 0.5);
    std::vector<int> addresses;
    auto transfer = [&addresses](int function, int address, std::vector<int>& data) {
        EXPECT_EQ(function, 0x10);
        EXPECT_LE(data.size(), 3u);
        addresses.push_back(address);
        return true;
    };
    ASSERT_TRUE(erg32::writeMotion(rotated, last, transfer));
    EXPECT_EQ(addresses, (std::vector<int>{0x03EC, 0x03EF}));
    addresses.clear();
    ASSERT_TRUE(erg32::writeMotion(erg32::encode({erg32::pi / 2, 0.016}, 0.5, 0.5), last, transfer));
    EXPECT_EQ(addresses, (std::vector<int>{0x03EA}));
    addresses.clear();
    ASSERT_TRUE(erg32::writeMotion(last, last, transfer));
    EXPECT_TRUE(addresses.empty());
}

TEST(Erg32Protocol, FailedParametersNeverTriggerAndDoNotAdvanceCache) {
    auto last = erg32::encode({0, 0}, 1, 1);
    const auto original = last;
    int calls = 0;
    EXPECT_FALSE(erg32::writeMotion(erg32::encode({1, 0}, 1, 1), last,
        [&calls](int, int address, std::vector<int>&) {
            ++calls; EXPECT_EQ(address, 0x03EC); return false;
        }));
    EXPECT_EQ(calls, 1);
    EXPECT_EQ(last, original);
}

TEST(Erg32Interfaces, ValidatesJointPairsAndExportsBothAxes) {
    Erg32Gripper g(rclcpp::get_logger("test"), {"right_hand_gripper_joint", "right_hand_rotate_joint"}, {});
    EXPECT_EQ(g.getJointCount(), 2u);
    EXPECT_EQ(g.getJointName(), "right_hand_rotate_joint");
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> states;
    std::vector<hardware_interface::CommandInterface::SharedPtr> commands;
    g.exportStateInterfaces(states); g.exportCommandInterfaces(commands);
    EXPECT_EQ(states.size(), 6u);
    ASSERT_EQ(commands.size(), 2u);
    EXPECT_EQ(commands[0]->get_name(), "right_hand_rotate_joint/position");
    EXPECT_EQ(commands[1]->get_name(), "right_hand_gripper_joint/position");
    EXPECT_TRUE(std::isnan(g.getEffortPtr()[0]));
    EXPECT_FALSE(g.writeCommandPeriodic());
    EXPECT_THROW(erg32::orderedJoints({"left_hand_rotate_joint", "right_hand_gripper_joint"}), std::invalid_argument);
    EXPECT_THROW(erg32::orderedJoints({"rotate_joint", "rotate_joint", "gripper_joint"}), std::invalid_argument);
}

TEST(Erg32Lifecycle, ReadyDeviceDoesNotResetOrTriggerMotionAndStopsOnExit) {
    Erg32Gripper g(rclcpp::get_logger("test"), {"rotate_joint", "gripper_joint"}, {});
    std::mutex mutex;
    std::vector<std::pair<int, std::vector<int>>> writes;
    Erg32TestPeer::transport(g, [&](int function, int address, std::vector<int>& data) {
        if (function == 3) {
            const std::array<uint16_t, 9> r{0x18, 0x18, 0, 0, 90, 0, 1, 0, 0x2818};
            for (size_t i = 0; i < data.size(); ++i) data[i] = r[address - 0x07D0 + i];
        } else {
            std::lock_guard<std::mutex> lock(mutex);
            writes.emplace_back(address, data);
        }
        return true;
    });
    ASSERT_TRUE(Erg32TestPeer::initialize(g));
    EXPECT_NEAR(g.getPositionCommandPtr()[0], erg32::pi / 2, 1e-12);
    EXPECT_DOUBLE_EQ(g.getPositionCommandPtr()[1], 0.016);
    EXPECT_TRUE(g.writeCommandPeriodic());
    g.stop();
    ASSERT_EQ(writes.size(), 6u);
    EXPECT_EQ(writes[0], std::make_pair(0x03E8, std::vector<int>({0x11, 0x11})));
    EXPECT_EQ(writes[1].first, 0x03EA);
    EXPECT_EQ(writes[1].second[1] & 255, 0); // no grip trigger
    EXPECT_EQ(writes[2].first, 0x03EC);
    EXPECT_EQ(writes[2].second[0], 90); // preserve measured rotation
    EXPECT_EQ(writes[3], std::make_pair(0x03EF, std::vector<int>({0})));
    EXPECT_EQ(writes[4], std::make_pair(0x03E8, std::vector<int>({1, 1})));
    EXPECT_EQ(writes[5], std::make_pair(0x03E8, std::vector<int>({0x11, 0x11})));
    EXPECT_FALSE(g.isInitialized());
}

TEST(Erg32Lifecycle, InitialFaultRefusesActivation) {
    Erg32Gripper g(rclcpp::get_logger("test"), {"rotate_joint", "gripper_joint"}, {});
    Erg32TestPeer::transport(g, [](int function, int address, std::vector<int>& data) {
        EXPECT_EQ(function, 3);
        std::fill(data.begin(), data.end(), 0);
        if (address == 0x07D0) data[0] = 0x0818;
        return true;
    });
    EXPECT_FALSE(Erg32TestPeer::initialize(g));
    EXPECT_FALSE(g.isInitialized());
}
TEST(Erg32Protocol, FailedTriggerLeavesRotationPendingForRetry) {
    auto last = erg32::encode({0, 0}, 1, 1);
    const auto original = last;
    std::vector<int> addresses;
    EXPECT_FALSE(erg32::writeMotion(erg32::encode({1, 0}, 1, 1), last,
        [&addresses](int, int address, std::vector<int>&) {
            addresses.push_back(address);
            return address != 0x03EF;
        }));
    EXPECT_EQ(addresses, (std::vector<int>{0x03EC, 0x03EF}));
    EXPECT_EQ(last, original);
}

TEST(Erg32Lifecycle, WaitsForBothAxesBeforeReleasingStop) {
    Erg32Gripper g(rclcpp::get_logger("test"), {"rotate_joint", "gripper_joint"}, {});
    int polls = 0;
    bool activated = false;
    Erg32TestPeer::transport(g, [&](int function, int address, std::vector<int>& data) {
        if (function == 3) {
            std::fill(data.begin(), data.end(), 0);
            if (address == 0x07D0) {
                ++polls;
                data[0] = polls < 3 ? 0x08 : 0x18;
                data[1] = 0x18;
            }
        } else if (address == 0x03E8 && data == std::vector<int>({1, 1})) {
            if (activated) { EXPECT_GE(polls, 3); }
            activated = true;
        }
        return true;
    });
    ASSERT_TRUE(Erg32TestPeer::initialize(g));
    g.stop();
    EXPECT_GE(polls, 3);
    EXPECT_TRUE(activated);
}

TEST(Erg32Lifecycle, ActivationReadFailureStopsBothAxes) {
    Erg32Gripper g(rclcpp::get_logger("test"), {"rotate_joint", "gripper_joint"}, {});
    int reads = 0;
    std::vector<int> controls;
    Erg32TestPeer::transport(g, [&](int function, int address, std::vector<int>& data) {
        if (function == 3) {
            if (++reads > 3) return false;
            std::fill(data.begin(), data.end(), 0);
        } else if (address == 0x03E8) {
            EXPECT_EQ(data[0], data[1]);
            controls.push_back(data[0]);
        }
        return true;
    });
    EXPECT_FALSE(Erg32TestPeer::initialize(g));
    EXPECT_EQ(controls, (std::vector<int>{0, 1, 0x11}));
    EXPECT_FALSE(g.isInitialized());
}
TEST(Erg32Worker, LostFeedbackLatchesFaultAndStops) {
    Erg32Gripper::Config config;
    config.read_interval_ms = 20;
    config.feedback_timeout_ms = 40;
    Erg32Gripper g(rclcpp::get_logger("test"), {"rotate_joint", "gripper_joint"}, config);
    int reads = 0;
    int stops = 0;
    Erg32TestPeer::transport(g, [&](int function, int address, std::vector<int>& data) {
        if (function == 3) {
            if (++reads > 3) return false;
            std::fill(data.begin(), data.end(), 0);
            if (address == 0x07D0) { data[0] = 0x18; data[1] = 0x18; }
        } else if (address == 0x03E8 && data[0] == 0x11) {
            ++stops;
        }
        return true;
    });
    ASSERT_TRUE(Erg32TestPeer::initialize(g));
    EXPECT_TRUE(Erg32TestPeer::waitFault(g));
    EXPECT_FALSE(g.readStatusPeriodic());
    EXPECT_FALSE(g.writeCommandPeriodic());
    g.stop();
    EXPECT_GE(stops, 2); // staging stop plus fault stop
}

TEST(Erg32Worker, PublishesOnlyThroughControlThreadSnapshotCopy) {
    Erg32Gripper g(rclcpp::get_logger("test"), {"rotate_joint", "gripper_joint"}, {});
    int reads = 0;
    Erg32TestPeer::transport(g, [&](int function, int address, std::vector<int>& data) {
        if (function == 3) {
            ++reads;
            std::fill(data.begin(), data.end(), 0);
            if (address == 0x07D0) { data[0] = 0x18; data[1] = 0x18; }
            if (address == 0x07D3 && reads > 3) data[1] = 90;
        }
        return true;
    });
    ASSERT_TRUE(Erg32TestPeer::initialize(g));
    EXPECT_TRUE(Erg32TestPeer::waitPosition(g, erg32::pi / 2));
    EXPECT_DOUBLE_EQ(g.getPositionPtr()[0], 0.0);
    EXPECT_TRUE(g.readStatusPeriodic());
    EXPECT_NEAR(g.getPositionPtr()[0], erg32::pi / 2, 1e-12);
    g.stop();
}
} // namespace rokae_ros2_control
