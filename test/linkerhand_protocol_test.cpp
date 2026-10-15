#include "rokae_ros2_control/linkerhand_protocol.h"

#include <gtest/gtest.h>
#include <limits>
#include <numeric>

namespace lh = rokae_ros2_control::linkerhand;

TEST(LinkerHandProtocol, ModelSelectionAndLimits) {
    for (const auto* type : {"o6", "o7", "l6"}) {
        EXPECT_TRUE(lh::isSupported(type));
        EXPECT_TRUE(lh::isSupported(std::string("linkerhand_") + type));
        EXPECT_EQ(lh::modelFor(type, true).registers,
                  lh::modelFor(std::string("linkerhand_") + type, true).registers);
    }
    EXPECT_FALSE(lh::isSupported("linkerhand_o8"));
    EXPECT_THROW(lh::modelFor("o8", false), std::invalid_argument);
    EXPECT_DOUBLE_EQ(lh::modelFor("o6", false).upper_limits[1], 1.30);
    EXPECT_DOUBLE_EQ(lh::modelFor("o6", true).upper_limits[1], 1.36);
    EXPECT_DOUBLE_EQ(lh::modelFor("l6", false).upper_limits[0], 0.99);
    EXPECT_DOUBLE_EQ(lh::modelFor("o7", true).upper_limits[0], 0.5146);
}

TEST(LinkerHandProtocol, O7ThumbMappingAndRegisterBlocks) {
    const auto model = lh::modelFor("o7", true);
    EXPECT_EQ(model.jointCount(), 7u);
    // URDF thumb_joint1 is roll, thumb_joint3 is pitch (matches USB-Modbus driver).
    EXPECT_EQ(model.registers, (std::vector<int>{6, 1, 0, 2, 3, 4, 5}));
    EXPECT_EQ(model.torqueRegister(), 7);
    EXPECT_EQ(model.speedRegister(), 14);
    for (const auto* type : {"o6", "l6"}) {
        const auto six = lh::modelFor(type, false);
        EXPECT_EQ(six.registers, (std::vector<int>{0, 1, 2, 3, 4, 5}));
        EXPECT_EQ(six.torqueRegister(), 6);
        EXPECT_EQ(six.speedRegister(), 12);
    }
}

TEST(LinkerHandProtocol, ConversionEndpointsClampingAndRoundTrip) {
    for (const auto* type : {"o6", "o7", "l6"}) {
        for (bool right : {false, true}) {
            for (double upper : lh::modelFor(type, right).upper_limits) {
                EXPECT_EQ(lh::positionToRegister(0.0, upper), 255);
                EXPECT_EQ(lh::positionToRegister(upper, upper), 0);
                EXPECT_EQ(lh::positionToRegister(-1.0, upper), 255);
                EXPECT_EQ(lh::positionToRegister(upper + 1.0, upper), 0);
                for (int raw = 0; raw <= 255; ++raw) {
                    EXPECT_EQ(lh::positionToRegister(lh::registerToPosition(raw, upper), upper), raw);
                }
            }
        }
    }
    EXPECT_THROW(lh::positionToRegister(std::numeric_limits<double>::quiet_NaN(), 1.0), std::invalid_argument);
    EXPECT_THROW(lh::positionToRegister(std::numeric_limits<double>::infinity(), 1.0), std::invalid_argument);
}

TEST(LinkerHandProtocol, JointSelectionRejectsMissingDuplicateAndMixedHands) {
    const auto model = lh::modelFor("o7", true);
    std::vector<std::string> names;
    for (const auto& suffix : model.suffixes) names.push_back("right_hand_" + suffix);
    const auto expected = names;
    std::reverse(names.begin(), names.end());
    names.push_back("right_joint1");
    EXPECT_EQ(lh::orderedJoints(model, names), expected);
    names.push_back(expected.front());
    EXPECT_THROW(lh::orderedJoints(model, names), std::invalid_argument);
    names = expected;
    names.pop_back();
    EXPECT_THROW(lh::orderedJoints(model, names), std::invalid_argument);
    names = expected;
    names.back() = "left_hand_pinky_joint";
    EXPECT_THROW(lh::orderedJoints(model, names), std::invalid_argument);
}

TEST(LinkerHandProtocol, SdkRequestsStayWithinThreeRegistersAndModelBlock) {
    for (const auto* type : {"o6", "o7", "l6"}) {
        const auto model = lh::modelFor(type, true);
        for (int start : {0, model.torqueRegister(), model.speedRegister()}) {
            std::vector<int> values(model.jointCount());
            std::iota(values.begin(), values.end(), 10);
            const auto expected = values;
            std::vector<int> addresses, counts;
            EXPECT_TRUE(lh::transferRegisters(start, values,
                [&](int address, std::vector<int>& chunk) {
                    addresses.push_back(address);
                    counts.push_back(static_cast<int>(chunk.size()));
                    for (size_t i = 0; i < chunk.size(); ++i) {
                        EXPECT_EQ(chunk[i], expected[address - start + i]);
                        chunk[i] = address + static_cast<int>(i);
                    }
                    return true;
                }));
            EXPECT_EQ(addresses[0], start);
            EXPECT_EQ(addresses[1], start + 3);
            if (model.jointCount() == 7) {
                EXPECT_EQ(addresses, (std::vector<int>{start, start + 3, start + 6}));
                EXPECT_EQ(counts, (std::vector<int>{3, 3, 1}));
            } else {
                EXPECT_EQ(counts, (std::vector<int>{3, 3}));
            }
            for (size_t i = 0; i < values.size(); ++i) EXPECT_EQ(values[i], start + static_cast<int>(i));
        }
    }
}

TEST(LinkerHandProtocol, TransferStopsOnFailure) {
    std::vector<int> values(7, 255);
    int calls = 0;
    EXPECT_FALSE(lh::transferRegisters(0, values,
        [&](int, std::vector<int>&) { return ++calls < 2; }));
    EXPECT_EQ(calls, 2);
}
