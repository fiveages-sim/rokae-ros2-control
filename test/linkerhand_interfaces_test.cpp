#include "rokae_ros2_control/linkerhand.h"
#include "rokae_ros2_control/linkerhand_l6.h"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

TEST(LinkerHandInterfaces, ExportsAllModelJointsWithoutConnectingToRobot) {
    for (const auto* type : {"o6", "o7", "l6"}) {
        for (bool right : {false, true}) {
            const auto model = rokae_ros2_control::linkerhand::modelFor(type, right);
            std::vector<std::string> names;
            for (const auto& suffix : model.suffixes) {
                names.push_back(std::string(right ? "right_hand_" : "left_hand_") + suffix);
            }
            rokae_ros2_control::LinkerHand::Config config;
            config.hand_type = type;
            config.right_hand = right;
            rokae_ros2_control::LinkerHand hand(rclcpp::get_logger("test"),
                std::make_shared<rclcpp::Clock>(), names, config);
            EXPECT_TRUE(hand.hasGripper());
            EXPECT_EQ(hand.getJointCount(), names.size());
            EXPECT_FALSE(hand.isInitialized());
            EXPECT_FALSE(hand.writeCommandPeriodic());
            std::vector<hardware_interface::StateInterface::ConstSharedPtr> states;
            std::vector<hardware_interface::CommandInterface::SharedPtr> commands;
            hand.exportStateInterfaces(states);
            hand.exportCommandInterfaces(commands);
            ASSERT_EQ(states.size(), 3 * names.size());
            ASSERT_EQ(commands.size(), names.size());
            for (size_t i = 0; i < names.size(); ++i) {
                EXPECT_EQ(commands[i]->get_name(), names[i] + "/position");
                EXPECT_EQ(states[3 * i]->get_name(), names[i] + "/position");
                EXPECT_DOUBLE_EQ(hand.getPositionCommandPtr()[i], 0.0);
                EXPECT_DOUBLE_EQ(hand.getPositionPtr()[i], 0.0);
            }
        }
    }
}

TEST(LinkerHandInterfaces, OldL6HeaderKeepsDefaultModel) {
    rokae_ros2_control::LinkerHandL6::Config config;
    EXPECT_EQ(config.hand_type, "linkerhand_l6");
}
