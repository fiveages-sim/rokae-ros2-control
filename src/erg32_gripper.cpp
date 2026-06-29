#include "rokae_ros2_control/erg32_gripper.h"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <system_error>

namespace rokae_ros2_control {

std::vector<std::string> Erg32Gripper::detectJoints(
    const std::vector<hardware_interface::ComponentInfo>& joints) {
    std::string rotate_joint;
    std::string gripper_joint;
    for (const auto& joint : joints) {
        std::string lower = joint.name;
        std::transform(lower.begin(), lower.end(), lower.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (lower.find("rotate") != std::string::npos) {
            rotate_joint = joint.name;
        }
        if (lower.find("gripper") != std::string::npos) {
            gripper_joint = joint.name;
        }
    }
    if (rotate_joint.empty() || gripper_joint.empty()) {
        return {};
    }
    return {rotate_joint, gripper_joint};
}

Erg32Gripper::Erg32Gripper(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    std::vector<std::string> joint_names,
    Config config)
    : logger_(std::move(logger)),
      clock_(std::move(clock)),
      joint_names_(std::move(joint_names)),
      config_(std::move(config)),
      torque_(std::clamp(config_.normalized_torque, 0.0, 1.0)),
      velocity_(std::clamp(config_.normalized_velocity, 0.0, 1.0)) {}

Erg32Gripper::~Erg32Gripper() {
    running_ = false;
    cv_.notify_all();
    if (communication_thread_.joinable()) {
        communication_thread_.join();
    }
}

uint8_t Erg32Gripper::toByte(const double normalized) {
    return static_cast<uint8_t>(std::lround(std::clamp(normalized, 0.0, 1.0) * 255.0));
}

void Erg32Gripper::setToolDynamics(const double normalized_torque, const double normalized_velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double next_torque = std::clamp(normalized_torque, 0.0, 1.0);
    const double next_velocity = std::clamp(normalized_velocity, 0.0, 1.0);
    if (next_torque == torque_ && next_velocity == velocity_) {
        return;
    }
    torque_ = next_torque;
    velocity_ = next_velocity;
    dynamics_dirty_ = true;
}

bool Erg32Gripper::writeRegisters(const uint16_t reg_addr, const std::vector<int>& values) {
    if (robot_ == nullptr || values.empty()) {
        return false;
    }

    for (size_t offset = 0; offset < values.size(); offset += REGISTERS_PER_REQUEST) {
        const int count = static_cast<int>(
            std::min(static_cast<size_t>(REGISTERS_PER_REQUEST), values.size() - offset));
        std::vector<int> chunk(values.begin() + static_cast<std::ptrdiff_t>(offset),
                               values.begin() + static_cast<std::ptrdiff_t>(offset + count));
        std::error_code ec;
        robot_->XPRWModbusRTUReg(SLAVE_ADDRESS, WRITE_FUNCTION,
                                reg_addr + static_cast<uint16_t>(offset), "int16",
                                count, chunk, false, ec);
        if (ec) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                                 "ERG32 write failed at register 0x%04X: %s",
                                 reg_addr + static_cast<uint16_t>(offset), ec.message().c_str());
            return false;
        }
    }
    return true;
}

bool Erg32Gripper::initialize(rokae::xMateErProRobot& robot) {
    if (joint_names_.size() != JOINT_COUNT) {
        RCLCPP_ERROR(logger_, "ERG32 requires 2 joints (rotate + gripper), got %zu",
                     joint_names_.size());
        return false;
    }

    robot_ = &robot;

    const std::vector<int> activate_values(CTRL_REG_COUNT, ACTIVATE_VALUE);
    if (!writeRegisters(CTRL_REG_BASE, activate_values)) {
        return false;
    }

    if (!readStatus()) {
        RCLCPP_WARN(logger_, "Unable to read initial ERG32 state; continuing with zero positions");
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        commands_ = positions_;
        pending_commands_ = positions_;
        last_commands_ = positions_;
    }

    initialized_ = true;
    running_ = true;
    communication_thread_ = std::thread(&Erg32Gripper::communicationLoop, this);
    RCLCPP_INFO(logger_, "ERG32 initialized (joints: %s, %s)",
                joint_names_[0].c_str(), joint_names_[1].c_str());
    return true;
}

bool Erg32Gripper::readStatus() {
    if (robot_ == nullptr) {
        return false;
    }

    std::vector<int> values(STATUS_REG_COUNT, 0);
    for (int offset = 0; offset < STATUS_REG_COUNT; offset += REGISTERS_PER_REQUEST) {
        const int count = std::min(REGISTERS_PER_REQUEST, STATUS_REG_COUNT - offset);
        std::vector<int> chunk(count, 0);
        std::error_code ec;
        robot_->XPRWModbusRTUReg(SLAVE_ADDRESS, READ_FUNCTION,
                                STATUS_REG_ADDR + static_cast<uint16_t>(offset),
                                "int16", count, chunk, false, ec);
        if (ec) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                                 "ERG32 read failed at register 0x%04X: %s",
                                 STATUS_REG_ADDR + static_cast<uint16_t>(offset),
                                 ec.message().c_str());
            return false;
        }
        std::copy(chunk.begin(), chunk.end(), values.begin() + offset);
    }

    if (values.size() < 5) {
        return false;
    }

    const uint8_t grip_raw = static_cast<uint8_t>(values[2] & 0xFF);
    const double grip_norm = 1.0 - (static_cast<double>(grip_raw) / 255.0);
    const double grip_m = std::clamp(grip_norm, 0.0, 1.0) * MAX_GRIP_OPENING_M;
    const int16_t rotate_deg = static_cast<int16_t>(values[4]);
    const double rotate_rad = rotate_deg * kPi / 180.0;

    std::lock_guard<std::mutex> lock(mutex_);
    positions_[0] = rotate_rad;
    positions_[1] = grip_m;
    return true;
}

bool Erg32Gripper::writeCommand(const std::array<double, JOINT_COUNT>& command) {
    const double rotate_rad = std::clamp(command[0], -kPi, kPi);
    const double grip_m = std::clamp(command[1], 0.0, MAX_GRIP_OPENING_M);
    const int16_t rot_deg = static_cast<int16_t>(std::lround(rotate_rad * 180.0 / kPi));

    double torque = 1.0;
    double velocity = 1.0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        torque = torque_;
        velocity = velocity_;
    }

    const double grip_norm = grip_m / MAX_GRIP_OPENING_M;
    const uint8_t grip_pos = static_cast<uint8_t>(
        255 - std::lround(std::clamp(grip_norm, 0.0, 1.0) * 255.0));
    const uint8_t grip_vel = toByte(velocity);
    const uint8_t grip_trq = toByte(torque);
    const uint8_t rot_vel = toByte(velocity);
    const uint8_t rot_trq = toByte(torque);

    std::vector<int> block(6, 0);
    block[0] = static_cast<int>(grip_pos | (static_cast<uint16_t>(grip_vel) << 8));
    block[1] = static_cast<int>(GRIP_TRIGGER | (static_cast<uint16_t>(grip_trq) << 8));
    block[2] = static_cast<int>(rot_deg);
    block[3] = static_cast<int>(rot_vel | (static_cast<uint16_t>(rot_trq) << 8));
    block[4] = 0;
    block[5] = static_cast<int>(ROTATE_ABS_TRIGGER);

    return writeRegisters(GRIP_PARAM_REG, block);
}

bool Erg32Gripper::writeCommandPeriodic() {
    if (!initialized_) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    bool changed = dynamics_dirty_;
    constexpr double rotate_threshold = 0.01;
    constexpr double grip_threshold = MAX_GRIP_OPENING_M * 0.01;
    for (size_t i = 0; i < JOINT_COUNT; ++i) {
        const double threshold = (i == 0) ? rotate_threshold : grip_threshold;
        changed = changed ||
                  std::abs(commands_[i] - last_commands_[i]) > threshold;
    }
    if (!changed) {
        return false;
    }

    pending_commands_ = commands_;
    command_pending_ = true;
    cv_.notify_one();
    return true;
}

void Erg32Gripper::communicationLoop() {
    auto next_read = std::chrono::steady_clock::now();
    while (running_) {
        std::array<double, JOINT_COUNT> command{};
        bool send = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_until(lock, next_read, [this] { return !running_ || command_pending_; });
            if (!running_) {
                break;
            }
            if (command_pending_) {
                command = pending_commands_;
                command_pending_ = false;
                send = true;
            }
        }

        if (send && writeCommand(command)) {
            std::lock_guard<std::mutex> lock(mutex_);
            last_commands_ = command;
            dynamics_dirty_ = false;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= next_read) {
            readStatus();
            next_read = now + std::chrono::milliseconds(READ_INTERVAL_MS);
        }
    }
}

void Erg32Gripper::exportStateInterfaces(
    std::vector<hardware_interface::StateInterface::ConstSharedPtr>& state_interfaces) {
    for (size_t i = 0; i < JOINT_COUNT; ++i) {
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &positions_[i]));
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocities_[i]));
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names_[i], hardware_interface::HW_IF_EFFORT, &efforts_[i]));
    }
}

void Erg32Gripper::exportCommandInterfaces(
    std::vector<hardware_interface::CommandInterface::SharedPtr>& command_interfaces) {
    for (size_t i = 0; i < JOINT_COUNT; ++i) {
        command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &commands_[i]));
    }
}

} // namespace rokae_ros2_control
