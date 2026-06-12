#pragma once

#include "rokae/robot.h"

#include <hardware_interface/handle.hpp>

#include <memory>
#include <string>
#include <vector>

namespace rokae_ros2_control {

class RokaeGripperInterface {
public:
    virtual ~RokaeGripperInterface() = default;

    virtual bool initialize(rokae::xMateErProRobot& robot) = 0;
    virtual bool readStatusPeriodic() = 0;
    virtual bool writeCommandPeriodic() = 0;

    virtual bool hasGripper() const = 0;
    virtual const std::string& getJointName() const = 0;
    virtual bool isInitialized() const = 0;

    virtual double* getPositionPtr() = 0;
    virtual double* getVelocityPtr() = 0;
    virtual double* getEffortPtr() = 0;
    virtual double* getPositionCommandPtr() = 0;

    virtual void setToolDynamics(double normalized_torque, double normalized_velocity) = 0;

    virtual void exportStateInterfaces(
        std::vector<hardware_interface::StateInterface::ConstSharedPtr>& state_interfaces) = 0;
    virtual void exportCommandInterfaces(
        std::vector<hardware_interface::CommandInterface::SharedPtr>& command_interfaces) = 0;
};

} // namespace rokae_ros2_control
