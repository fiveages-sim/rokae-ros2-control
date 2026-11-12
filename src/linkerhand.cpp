#include "rokae_ros2_control/linkerhand.h"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace rokae_ros2_control {

LinkerHand::LinkerHand(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    std::vector<std::string> joint_names,
    Config config)
    : logger_(std::move(logger)),
      clock_(std::move(clock)),
      joint_names_(std::move(joint_names)),
      config_(std::move(config)),
      model_(linkerhand::modelFor(config_.hand_type, config_.right_hand)),
      positions_(model_.jointCount(), 0.0),
      velocities_(model_.jointCount(), 0.0),
      efforts_(model_.jointCount(), 0.0),
      commands_(model_.jointCount(), 0.0),
      pending_commands_(model_.jointCount(), 0.0),
      last_commands_(model_.jointCount(), std::numeric_limits<double>::quiet_NaN()),
      torque_(std::clamp(config_.normalized_torque, 0.0, 1.0)),
      velocity_(std::clamp(config_.normalized_velocity, 0.0, 1.0)) {
    if (joint_names_ != linkerhand::orderedJoints(model_, joint_names_)) {
        throw std::invalid_argument("LinkerHand joints must be in ROS order");
    }
}

LinkerHand::~LinkerHand() { stop(); }

void LinkerHand::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        initialized_ = false;
        command_pending_ = false;
    }
    cv_.notify_all();
    if (communication_thread_.joinable()) {
        communication_thread_.join();
    }
    robot_ = nullptr;
    have_valid_positions_ = false;
}

bool LinkerHand::initialize(rokae::xMateErProRobot& robot) {
    stop();
    if (joint_names_.size() != model_.jointCount()) {
        RCLCPP_ERROR(logger_, "LinkerHand %s requires %zu joints, got %zu", model_.name.c_str(), model_.jointCount(), joint_names_.size());
        return false;
    }

    robot_ = &robot;
    if (config_.configure_xpanel) {
        std::error_code ec;
        robot.setxPanelRS485(rokae::xPanelOpt::Vout::supply24v, true, ec);
        if (ec) {
            RCLCPP_ERROR(logger_, "Failed to enable Rokae xPanel RS485 for LinkerHand: %s", ec.message().c_str());
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (!readPositions()) {
        RCLCPP_ERROR(logger_, "Failed to read initial LinkerHand position");
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
    try {
        communication_thread_ = std::thread(&LinkerHand::communicationLoop, this);
    } catch (const std::exception& e) {
        stop();
        RCLCPP_ERROR(logger_, "Failed to start LinkerHand worker: %s", e.what());
        return false;
    }
    RCLCPP_INFO(logger_, "LinkerHand %s initialized (slave=0x%02X, joints=%zu)", model_.name.c_str(), config_.slave_id, model_.jointCount());
    return true;
}

void LinkerHand::setToolDynamics(double normalized_torque, double normalized_velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    torque_ = std::clamp(normalized_torque, 0.0, 1.0);
    velocity_ = std::clamp(normalized_velocity, 0.0, 1.0);
    dynamics_dirty_ = true;
}

bool LinkerHand::writeCommandPeriodic() {
    if (!initialized_) return false;

    std::lock_guard<std::mutex> lock(mutex_);
    bool changed = dynamics_dirty_;
    for (size_t i = 0; i < model_.jointCount(); ++i) {
        if (!std::isfinite(commands_[i])) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                                 "Ignoring invalid LinkerHand command for %s: %.6f",
                                 joint_names_[i].c_str(), commands_[i]);
            commands_[i] = last_commands_[i];
            continue;
        }

        const double upper_limit = model_.upper_limits[i];
        const double clamped = std::clamp(commands_[i], 0.0, upper_limit);
        if (std::abs(clamped - commands_[i]) > 1e-6) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                                 "Clamped LinkerHand command for %s from %.6f to %.6f",
                                 joint_names_[i].c_str(), commands_[i], clamped);
            commands_[i] = clamped;
        }

        changed = changed || std::isnan(last_commands_[i]) ||
                  std::abs(commands_[i] - last_commands_[i]) > 0.001;
    }
    if (!changed) return false;

    pending_commands_ = commands_;
    command_pending_ = true;
    cv_.notify_one();
    return true;
}

bool LinkerHand::readPositions() {
    if (robot_ == nullptr) return false;
    std::vector<int> values(model_.jointCount(), 0);
    const bool ok = linkerhand::transferRegisters(POSITION_REG, values,
        [this](int address, std::vector<int>& chunk) {
            std::error_code ec;
            robot_->XPRWModbusRTUReg(config_.slave_id, READ_INPUT_REGS, address,
                                    "int16", static_cast<int>(chunk.size()), chunk, false, ec);
            if (ec) {
                RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                    "LinkerHand read failed at register %d: %s", address, ec.message().c_str());
            }
            return !ec;
        });
    if (!ok) return false;

    std::vector<double> new_positions(model_.jointCount());
    for (size_t i = 0; i < model_.jointCount(); ++i) {
        new_positions[i] = linkerhand::registerToPosition(values[model_.registers[i]], model_.upper_limits[i]);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < model_.jointCount(); ++i) {
        const double upper_limit = model_.upper_limits[i];
        const double jump_limit = maxFeedbackJump(upper_limit);
        const bool have_position = have_valid_positions_ && std::isfinite(positions_[i]);
        const bool near_command = std::isfinite(last_commands_[i]) &&
                                  std::abs(new_positions[i] - last_commands_[i]) <= jump_limit;
        const bool sudden_jump = have_position &&
                                 std::abs(new_positions[i] - positions_[i]) > jump_limit &&
                                 !near_command;

        if (sudden_jump) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                                 "Rejected LinkerHand feedback jump for %s: %.6f -> %.6f "
                                 "(last command %.6f, raw=%d)",
                                 joint_names_[i].c_str(), positions_[i], new_positions[i],
                                 last_commands_[i], values[model_.registers[i]]);
            continue;
        }

        positions_[i] = new_positions[i];
    }
    have_valid_positions_ = true;
    return true;
}

bool LinkerHand::writeCommand(const std::vector<double>& command) {
    std::vector<int> position_values(model_.jointCount());
    double torque;
    double velocity;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        torque = torque_;
        velocity = velocity_;
    }
    const int raw_torque = static_cast<int>(std::lround(torque * 255.0));
    const int raw_velocity = static_cast<int>(std::lround(velocity * 255.0));
    for (size_t i = 0; i < model_.jointCount(); ++i) {
        if (!std::isfinite(command[i])) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                                 "Refusing to write invalid LinkerHand command for %s: %.6f",
                                 joint_names_[i].c_str(), command[i]);
            return false;
        }
        position_values[model_.registers[i]] = linkerhand::positionToRegister(command[i], model_.upper_limits[i]);
    }

    const std::array<int, 3> block_starts = {POSITION_REG, model_.torqueRegister(), model_.speedRegister()};
    for (const int block_start : block_starts) {
        std::vector<int> values = block_start == POSITION_REG ? position_values :
            std::vector<int>(model_.jointCount(), block_start == model_.torqueRegister() ? raw_torque : raw_velocity);
        if (!linkerhand::transferRegisters(block_start, values,
            [this](int address, std::vector<int>& chunk) {
                std::error_code ec;
                robot_->XPRWModbusRTUReg(config_.slave_id, WRITE_MULTI_REGS, address,
                                        "int16", static_cast<int>(chunk.size()), chunk, false, ec);
                if (ec) {
                    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                        "LinkerHand write failed at register %d: %s", address, ec.message().c_str());
                }
                return !ec;
            })) return false;
    }
    return true;
}

void LinkerHand::communicationLoop() {
    auto next_read = std::chrono::steady_clock::now();
    while (running_) {
        std::vector<double> command;
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

double LinkerHand::maxFeedbackJump(double upper_limit) {
    return std::max(0.20, upper_limit * 0.35);
}

void LinkerHand::exportStateInterfaces(
    std::vector<hardware_interface::StateInterface::ConstSharedPtr>& state_interfaces) {
    for (size_t i = 0; i < model_.jointCount(); ++i) {
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &positions_[i]));
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocities_[i]));
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names_[i], hardware_interface::HW_IF_EFFORT, &efforts_[i]));
    }
}

void LinkerHand::exportCommandInterfaces(
    std::vector<hardware_interface::CommandInterface::SharedPtr>& command_interfaces) {
    for (size_t i = 0; i < model_.jointCount(); ++i) {
        command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &commands_[i]));
    }
}

} // namespace rokae_ros2_control
