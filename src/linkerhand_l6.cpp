#include "rokae_ros2_control/linkerhand_l6.h"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace rokae_ros2_control {

LinkerHandL6::LinkerHandL6(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    std::vector<std::string> joint_names,
    Config config)
    : logger_(std::move(logger)),
      clock_(std::move(clock)),
      joint_names_(std::move(joint_names)),
      config_(std::move(config)),
      torque_(std::clamp(config_.normalized_torque, 0.0, 1.0)),
      velocity_(std::clamp(config_.normalized_velocity, 0.0, 1.0)) {
    last_commands_.fill(std::numeric_limits<double>::quiet_NaN());
}

LinkerHandL6::~LinkerHandL6() {
    running_ = false;
    cv_.notify_all();
    if (communication_thread_.joinable()) {
        communication_thread_.join();
    }
}

bool LinkerHandL6::initialize(rokae::xMateErProRobot& robot) {
    if (joint_names_.size() != JOINT_COUNT) {
        RCLCPP_ERROR(logger_, "LinkerHand L6 requires 6 joints, got %zu", joint_names_.size());
        return false;
    }

    robot_ = &robot;
    if (config_.configure_xpanel) {
        std::error_code ec;
        robot.setxPanelRS485(rokae::xPanelOpt::Vout::supply24v, true, ec);
        if (ec) {
            RCLCPP_ERROR(logger_, "Failed to enable Rokae xPanel RS485 for L6: %s", ec.message().c_str());
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (!readPositions()) {
        RCLCPP_ERROR(logger_, "Failed to read initial LinkerHand L6 position");
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        commands_ = positions_;
        pending_commands_ = positions_;
        last_commands_ = positions_;
    }

    initialized_ = true;
    running_ = true;
    communication_thread_ = std::thread(&LinkerHandL6::communicationLoop, this);
    RCLCPP_INFO(logger_, "LinkerHand L6 initialized (slave=0x%02X, joints=6)", config_.slave_id);
    return true;
}

void LinkerHandL6::setToolDynamics(double normalized_torque, double normalized_velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    torque_ = std::clamp(normalized_torque, 0.0, 1.0);
    velocity_ = std::clamp(normalized_velocity, 0.0, 1.0);
    dynamics_dirty_ = true;
}

bool LinkerHandL6::writeCommandPeriodic() {
    if (!initialized_) return false;

    std::lock_guard<std::mutex> lock(mutex_);
    bool changed = dynamics_dirty_;
    for (size_t i = 0; i < JOINT_COUNT; ++i) {
        changed = changed || std::isnan(last_commands_[i]) ||
                  std::abs(commands_[i] - last_commands_[i]) > 0.001;
    }
    if (!changed) return false;

    pending_commands_ = commands_;
    command_pending_ = true;
    cv_.notify_one();
    return true;
}

bool LinkerHandL6::readPositions() {
    if (robot_ == nullptr) return false;
    std::vector<int> values(JOINT_COUNT, 0);
    for (int offset = 0; offset < static_cast<int>(JOINT_COUNT); offset += REGISTERS_PER_REQUEST) {
        std::vector<int> chunk(REGISTERS_PER_REQUEST, 0);
        std::error_code ec;
        robot_->XPRWModbusRTUReg(config_.slave_id, READ_INPUT_REGS, POSITION_REG + offset,
                                "int16", REGISTERS_PER_REQUEST, chunk, false, ec);
        if (ec) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                                 "LinkerHand L6 read failed at register %d: %s",
                                 POSITION_REG + offset, ec.message().c_str());
            return false;
        }
        std::copy(chunk.begin(), chunk.end(), values.begin() + offset);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < JOINT_COUNT; ++i) {
        positions_[i] = registerToPosition(values[i], jointUpperLimit(joint_names_[i]));
    }
    return true;
}

bool LinkerHandL6::writeCommand(const std::array<double, JOINT_COUNT>& command) {
    std::array<int, JOINT_COUNT> position_values{};
    double torque;
    double velocity;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        torque = torque_;
        velocity = velocity_;
    }
    const int raw_torque = static_cast<int>(std::lround(torque * 255.0));
    const int raw_velocity = static_cast<int>(std::lround(velocity * 255.0));
    for (size_t i = 0; i < JOINT_COUNT; ++i) {
        position_values[i] = positionToRegister(command[i], jointUpperLimit(joint_names_[i]));
    }

    const std::array<int, 3> block_starts = {POSITION_REG, TORQUE_REG, SPEED_REG};
    for (const int block_start : block_starts) {
        for (int offset = 0; offset < static_cast<int>(JOINT_COUNT); offset += REGISTERS_PER_REQUEST) {
            std::vector<int> chunk(REGISTERS_PER_REQUEST, 0);
            for (int i = 0; i < REGISTERS_PER_REQUEST; ++i) {
                if (block_start == POSITION_REG) chunk[i] = position_values[offset + i];
                if (block_start == TORQUE_REG) chunk[i] = raw_torque;
                if (block_start == SPEED_REG) chunk[i] = raw_velocity;
            }
            std::error_code ec;
            robot_->XPRWModbusRTUReg(config_.slave_id, WRITE_MULTI_REGS, block_start + offset,
                                    "int16", REGISTERS_PER_REQUEST, chunk, false, ec);
            if (ec) {
                RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                                     "LinkerHand L6 write failed at register %d: %s",
                                     block_start + offset, ec.message().c_str());
                return false;
            }
        }
    }
    return true;
}

void LinkerHandL6::communicationLoop() {
    auto next_read = std::chrono::steady_clock::now();
    while (running_) {
        std::array<double, JOINT_COUNT> command{};
        bool send = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_until(lock, next_read, [this] { return !running_ || command_pending_; });
            if (!running_) break;
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
            readPositions();
            next_read = now + std::chrono::milliseconds(std::max(20, config_.read_interval_ms));
        }
    }
}

double LinkerHandL6::jointUpperLimit(const std::string& joint_name) {
    if (joint_name.find("thumb_joint1") != std::string::npos) return 0.99;
    if (joint_name.find("thumb_joint2") != std::string::npos) return 1.39;
    return 1.26;
}

int LinkerHandL6::positionToRegister(double position, double upper_limit) {
    const double normalized = std::clamp(position / upper_limit, 0.0, 1.0);
    return 255 - static_cast<int>(std::lround(normalized * 255.0));
}

double LinkerHandL6::registerToPosition(int value, double upper_limit) {
    const int raw = std::clamp(value & 0xFF, 0, 255);
    return (255.0 - static_cast<double>(raw)) / 255.0 * upper_limit;
}

void LinkerHandL6::exportStateInterfaces(
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

void LinkerHandL6::exportCommandInterfaces(
    std::vector<hardware_interface::CommandInterface::SharedPtr>& command_interfaces) {
    for (size_t i = 0; i < JOINT_COUNT; ++i) {
        command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &commands_[i]));
    }
}

} // namespace rokae_ros2_control
