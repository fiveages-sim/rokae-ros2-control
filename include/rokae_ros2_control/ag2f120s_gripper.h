#pragma once

#include "rokae_ros2_control/ag2f90d_gripper.h"

namespace rokae_ros2_control {

/** AG2F120S / AG2F120S_D — 120S 子类，参数取自 gripper_hardware_common::ModbusConfig。 */
class Ag2f120sGripper : public Ag2f90dGripper {
public:
    Ag2f120sGripper(
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock,
        std::string joint_name,
        Config config);

    void setToolDynamics(double normalized_torque, double normalized_velocity) override;

    static Config makeConfig120S();
    static Config makeConfig120SD();
    static Config makeConfig(bool is_d_variant);

private:
    static Config scaleConfigForBase(Config config, int max_torque, int max_velocity);

    int max_torque_value_;
    int max_velocity_value_;
};

} // namespace rokae_ros2_control
