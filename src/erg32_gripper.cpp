#include "rokae_ros2_control/erg32_gripper.h"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace rokae_ros2_control {
Erg32Gripper::Erg32Gripper(rclcpp::Logger logger, std::vector<std::string> joints, Config config)
    : logger_(std::move(logger)), joints_(erg32::orderedJoints(joints)), config_(config),
      torque_(config.normalized_torque), velocity_(config.normalized_velocity) {
    if (joints.size() != 2 || config.slave_id < 1 || config.slave_id > 247 ||
        config.read_interval_ms < 20 || config.feedback_timeout_ms <= config.read_interval_ms ||
        config.activation_timeout_ms <= 0) throw std::invalid_argument("Invalid ERG32 configuration");
    erg32::byte(torque_); erg32::byte(velocity_);
}

Erg32Gripper::~Erg32Gripper() { stop(); }

bool Erg32Gripper::initialize(rokae::xMateErProRobot& robot) {
    stop();
    transfer_ = [this, &robot](int function, int address, std::vector<int>& values) {
        std::error_code ec;
        robot.XPRWModbusRTUReg(config_.slave_id, function, address, "uint16",
                              static_cast<int>(values.size()), values, false, ec);
        if (ec) RCLCPP_WARN(logger_, "ERG32 Modbus failed at 0x%04X: %s", address, ec.message().c_str());
        return !ec;
    };
    if (config_.configure_xpanel) {
        std::error_code ec;
        robot.setxPanelRS485(rokae::xPanelOpt::Vout::supply24v, true, ec);
        if (ec) return false;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return initializeDevice();
}

bool Erg32Gripper::readFeedback(erg32::Feedback& feedback) {
    std::array<uint16_t, 9> registers{};
    for (int offset = 0; offset < 9; offset += 3) {
        std::vector<int> values(3, -1);
        if (!transfer_(0x03, 0x07D0 + offset, values) || values.size() != 3) return false;
        // uint16 transfers must fill every element. Do not accept an unchanged
        // output buffer as an all-zero device status when the bridge returns no data.
        for (int i = 0; i < 3; ++i) {
            if (values[i] < 0 || values[i] > 0xFFFF) {
                RCLCPP_WARN(logger_, "ERG32 incomplete/invalid uint16 read at 0x%04X: value=%d", 0x07D0 + offset + i, values[i]);
                return false;
            }
            registers[offset + i] = static_cast<uint16_t>(values[i]);
        }
    }
    feedback = erg32::decode(registers);
    return true;
}

bool Erg32Gripper::initializeDevice() {
    faulted_ = false;
    erg32::Feedback feedback;
    // Do not reset an already ready device: reset/activation may physically move both axes.
    if (!readFeedback(feedback) || !feedback.healthy()) {
        RCLCPP_ERROR(logger_, "ERG32 initial status unavailable or faulted; activation refused");
        return false;
    }
    const auto abort_activation = [this]() {
        std::vector<int> values{0x11, 0x11};
        transfer_(0x10, 0x03E8, values);
        return false;
    };
    if (!feedback.ready()) {
        std::vector<int> reset{0, 0}, activate{1, 1};
        if (!transfer_(0x10, 0x03E8, reset) || !transfer_(0x10, 0x03E8, activate)) return abort_activation();
        const auto deadline = Clock::now() + std::chrono::milliseconds(config_.activation_timeout_ms);
        while (true) {
            if (!readFeedback(feedback) || !feedback.healthy()) return abort_activation();
            if (feedback.ready()) break;
            if (Clock::now() >= deadline) {
                RCLCPP_ERROR(logger_, "ERG32 activation timed out");
                return abort_activation();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    if ((feedback.turns != 0 || std::abs(feedback.position[0]) > erg32::pi + 1e-6)) {
        RCLCPP_ERROR(logger_, "ERG32 position is outside the configured +/-180 degree range");
        return abort_activation();
    }
    // Stage current-position parameters with both triggers cleared before releasing
    // rSTOP. In particular, do not resume a pre-deactivation target on reactivation.
    std::vector<int> stop_values{0x11, 0x11};
    if (!transfer_(0x10, 0x03E8, stop_values)) return abort_activation();
    auto hold = erg32::encode(feedback.position, torque_, velocity_);
    hold[1] &= 0xFF00;
    hold[5] = 0;
    erg32::Packet unknown;
    unknown.fill(0xFFFF);
    if (!erg32::writeMotion(hold, unknown, transfer_)) return abort_activation();
    std::vector<int> enable{1, 1};
    if (!transfer_(0x10, 0x03E8, enable)) return abort_activation();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        feedback_ = feedback;
        positions_ = commands_ = feedback.position;
        velocities_ = {0, 0};
        feedback_time_ = published_time_ = Clock::now();
        pending_ = last_ = erg32::encode(commands_, torque_, velocity_);
        command_pending_ = false;
    }
    initialized_ = true;
    running_ = true;
    try {
        worker_ = std::thread(&Erg32Gripper::communicationLoop, this);
    } catch (...) {
        running_ = false;
        initialized_ = false;
        std::vector<int> stop_values{0x11, 0x11};
        transfer_(0x10, 0x03E8, stop_values);
        return false;
    }
    return true;
}

void Erg32Gripper::stop() {
    running_ = false;
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (initialized_ && transfer_) {
        std::vector<int> values{0x11, 0x11}; // rACT remains set, rSTOP stops both axes.
        transfer_(0x10, 0x03E8, values);
    }
    initialized_ = false;
    transfer_ = {};
}

void Erg32Gripper::fail(const char* reason) {
    faulted_ = true;
    running_ = false;
    RCLCPP_ERROR(logger_, "ERG32 fault latched: %s", reason);
    std::vector<int> values{0x11, 0x11};
    transfer_(0x10, 0x03E8, values);
}

bool Erg32Gripper::readStatusPeriodic() {
    if (!initialized_ || faulted_) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (Clock::now() - feedback_time_ > std::chrono::milliseconds(config_.feedback_timeout_ms)) return false;
    if (feedback_time_ != published_time_) {
        const double dt = std::chrono::duration<double>(feedback_time_ - published_time_).count();
        for (size_t i = 0; i < 2; ++i) velocities_[i] = (feedback_.position[i] - positions_[i]) / dt;
        positions_ = feedback_.position;
        published_time_ = feedback_time_;
    }
    return true;
}

bool Erg32Gripper::writeCommandPeriodic() {
    if (!initialized_ || faulted_) return false;
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        // Reject NaN/Inf before clamping. Invalid commands do not replace a valid pending target.
        const auto packet = erg32::encode(commands_, torque_, velocity_);
        commands_[0] = std::clamp(commands_[0], -erg32::pi, erg32::pi);
        commands_[1] = std::clamp(commands_[1], 0.0, erg32::finger_travel);
        if (packet != pending_) {
            pending_ = packet;
            command_pending_ = true;
            cv_.notify_one();
        }
        return true;
    } catch (const std::invalid_argument& e) {
        RCLCPP_ERROR(logger_, "%s", e.what());
        return false;
    }
}

void Erg32Gripper::setToolDynamics(double torque, double velocity) {
    erg32::byte(torque); erg32::byte(velocity);
    std::lock_guard<std::mutex> lock(mutex_);
    torque_ = std::clamp(torque, 0.0, 1.0);
    velocity_ = std::clamp(velocity, 0.0, 1.0);
}

void Erg32Gripper::communicationLoop() {
    try {
        auto next_read = Clock::now();
        bool allow_commands = true;
        const auto invalid_state = [](const erg32::Feedback& f) {
            return !f.ready() || f.turns != 0 || std::abs(f.position[0]) > erg32::pi + 1e-6;
        };
        const auto log_snapshot = [this](const char* label, const erg32::Feedback& f) {
            const auto& r = f.registers;
            RCLCPP_WARN(logger_,
                "ERG32 %s: status rotate=0x%02X grip=0x%02X; faults rotate=0x%02X grip=0x%02X; "
                "angle=%.1fdeg turns=%d; raw[07D0..07D8]=%04X %04X %04X %04X %04X %04X %04X %04X %04X",
                label, f.status[0], f.status[1], f.fault[0], f.fault[1],
                f.position[0] * 180.0 / erg32::pi, f.turns,
                r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);
        };
        while (running_) {
            erg32::Packet packet;
            bool send = false;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait_until(lock, next_read, [this, &allow_commands] {
                    return !running_ || (allow_commands && command_pending_);
                });
                if (!running_) break;
                if (allow_commands && command_pending_) {
                    packet = pending_;
                    command_pending_ = false;
                    send = true;
                }
            }
            if (send && !erg32::writeMotion(packet, last_, transfer_)) {
                fail("motion transfer failed"); break;
            }
            if (Clock::now() >= next_read) {
                erg32::Feedback feedback;
                bool valid_read = readFeedback(feedback);
                // Fault bits are authoritative. For state/range anomalies, confirm
                // with a fresh complete transaction before latching a terminal fault.
                // No commands or state publication occur between these two reads.
                if (valid_read && feedback.healthy() && invalid_state(feedback)) {
                    log_snapshot("unconfirmed state; rereading", feedback);
                    valid_read = readFeedback(feedback);
                    allow_commands = false;
                    if (valid_read && feedback.healthy() && !invalid_state(feedback)) {
                        RCLCPP_WARN(logger_, "ERG32 anomalous snapshot discarded; fresh read confirms ready state");
                    }
                }
                if (valid_read) {
                    if (!feedback.healthy() || invalid_state(feedback)) {
                        log_snapshot("confirmed invalid state", feedback);
                        if (!feedback.healthy()) fail("device fault bits set");
                        else if (!feedback.ready()) fail("activation lost on consecutive full reads");
                        else if (feedback.turns != 0) fail("nonzero turn count confirmed in single-turn mode");
                        else fail("rotation outside +/-180 degrees confirmed");
                        break;
                    }
                    allow_commands = true;
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (feedback.status != feedback_.status) {
                        RCLCPP_INFO(logger_, "ERG32 object state rotate=%u grip=%u dropped=%d (voltage=%uV temperature=%uC)",
                                    feedback.object[0], feedback.object[1], feedback.dropped, feedback.voltage, feedback.temperature);
                    }
                    feedback_ = feedback;
                    feedback_time_ = Clock::now();
                }
                bool stale;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    stale = Clock::now() - feedback_time_ > std::chrono::milliseconds(config_.feedback_timeout_ms);
                }
                if (stale) { fail("feedback timed out"); break; }
                next_read = Clock::now() + std::chrono::milliseconds(config_.read_interval_ms);
            }
        }
    } catch (const std::exception& e) {
        fail(e.what());
    }
}

void Erg32Gripper::exportStateInterfaces(std::vector<hardware_interface::StateInterface::ConstSharedPtr>& interfaces) {
    for (size_t i = 0; i < 2; ++i) {
        interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(joints_[i], "position", &positions_[i]));
        interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(joints_[i], "velocity", &velocities_[i]));
        interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(joints_[i], "effort", &efforts_[i]));
    }
}
void Erg32Gripper::exportCommandInterfaces(std::vector<hardware_interface::CommandInterface::SharedPtr>& interfaces) {
    for (size_t i = 0; i < 2; ++i) interfaces.push_back(
        std::make_shared<hardware_interface::CommandInterface>(joints_[i], "position", &commands_[i]));
}
} // namespace rokae_ros2_control
