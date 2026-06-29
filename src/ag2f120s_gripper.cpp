#include "rokae_ros2_control/ag2f120s_gripper.h"

#include "gripper_hardware_common/utils/ModbusConfig.h"

namespace rokae_ros2_control {

namespace {

constexpr int kAg2f90dTorqueVelocityScale = 255;

template <typename Traits>
Ag2f90dGripper::Config configFromTraits(const char* model_name, double max_opening_distance) {
    Ag2f90dGripper::Config config;
    config.slave_id = static_cast<int>(Traits::SLAVE_ID);
    config.close_pos = static_cast<int>(Traits::MAX_POSITION_MM);
    config.max_opening_distance = max_opening_distance;
    config.feedback_reg = static_cast<int>(Traits::FEEDBACK_REG_ADDR);
    config.velocity_reg = static_cast<int>(Traits::VELOCITY_REG);
    config.torque_reg = static_cast<int>(Traits::TORQUE_REG);
    config.acceleration_reg = static_cast<int>(Traits::ACCELERATION_REG);
    config.deceleration_reg = static_cast<int>(Traits::DECELERATION_REG);
    config.default_acceleration = static_cast<int>(Traits::DEFAULT_ACCELERATION);
    config.default_deceleration = static_cast<int>(Traits::DEFAULT_DECELERATION);
    config.model_name = model_name;
    return config;
}

} // namespace

Ag2f90dGripper::Config Ag2f120sGripper::scaleConfigForBase(
    Config config,
    int max_torque,
    int max_velocity) {
    config.normalized_torque =
        config.normalized_torque * static_cast<double>(max_torque) / kAg2f90dTorqueVelocityScale;
    config.normalized_velocity =
        config.normalized_velocity * static_cast<double>(max_velocity) / kAg2f90dTorqueVelocityScale;
    return config;
}

Ag2f120sGripper::Ag2f120sGripper(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    std::string joint_name,
    Config config)
    : Ag2f90dGripper(
          logger,
          clock,
          joint_name,
          scaleConfigForBase(
              config,
              config.model_name == "AG2F120S_D"
                  ? gripper_hardware_common::ModbusConfig::Changingtek120S_D::MAX_TORQUE_VALUE
                  : gripper_hardware_common::ModbusConfig::Changingtek120S::MAX_TORQUE_VALUE,
              config.model_name == "AG2F120S_D"
                  ? gripper_hardware_common::ModbusConfig::Changingtek120S_D::MAX_VELOCITY_VALUE
                  : gripper_hardware_common::ModbusConfig::Changingtek120S::MAX_VELOCITY_VALUE)),
      max_torque_value_(config.model_name == "AG2F120S_D"
                            ? gripper_hardware_common::ModbusConfig::Changingtek120S_D::MAX_TORQUE_VALUE
                            : gripper_hardware_common::ModbusConfig::Changingtek120S::MAX_TORQUE_VALUE),
      max_velocity_value_(config.model_name == "AG2F120S_D"
                              ? gripper_hardware_common::ModbusConfig::Changingtek120S_D::MAX_VELOCITY_VALUE
                              : gripper_hardware_common::ModbusConfig::Changingtek120S::MAX_VELOCITY_VALUE) {}

void Ag2f120sGripper::setToolDynamics(double normalized_torque, double normalized_velocity) {
    Ag2f90dGripper::setToolDynamics(
        normalized_torque * static_cast<double>(max_torque_value_) / kAg2f90dTorqueVelocityScale,
        normalized_velocity * static_cast<double>(max_velocity_value_) / kAg2f90dTorqueVelocityScale);
}

Ag2f90dGripper::Config Ag2f120sGripper::makeConfig120S() {
    using gripper_hardware_common::ModbusConfig::Changingtek120S;
    // 0=开、12000=关，反馈 0x060D，行程约 120 mm
    return configFromTraits<Changingtek120S>("AG2F120S", 0.12);
}

Ag2f90dGripper::Config Ag2f120sGripper::makeConfig120SD() {
    using gripper_hardware_common::ModbusConfig::Changingtek120S_D;
    // 0=开、1000=关，力矩/速度 0–100，反馈 0x0418，行程约 100 mm
    return configFromTraits<Changingtek120S_D>("AG2F120S_D", 0.10);
}

Ag2f90dGripper::Config Ag2f120sGripper::makeConfig(bool is_d_variant) {
    return is_d_variant ? makeConfig120SD() : makeConfig120S();
}

} // namespace rokae_ros2_control
