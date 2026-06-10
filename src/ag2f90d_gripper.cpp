#include "rokae_ros2_control/ag2f90d_gripper.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <system_error>
#include <utility>

namespace rokae_ros2_control {

Ag2f90dGripper::Ag2f90dGripper(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    std::string joint_name,
    Config config)
    : logger_(std::move(logger)),
      clock_(std::move(clock)),
      joint_name_(std::move(joint_name)),
      config_(config) {
    config_.close_pos = std::max(config_.close_pos, OPEN_POS);
    config_.max_opening_distance = std::max(config_.max_opening_distance, 0.001);
    config_.read_interval_ms = std::max(config_.read_interval_ms, 50);
    if (config_.model_name.empty()) {
        config_.model_name = "AG2F90D";
    }

    RCLCPP_INFO(logger_,
                "%s gripper configured: joint=%s, slave_id=%d, close_pos=%d, command_range=%.3f, feedback_reg=0x%04X, configure_xpanel=%s",
                config_.model_name.c_str(),
                joint_name_.c_str(),
                config_.slave_id,
                config_.close_pos,
                config_.max_opening_distance,
                config_.feedback_reg,
                config_.configure_xpanel ? "true" : "false");

    thread_running_ = true;
    command_thread_ = std::thread(&Ag2f90dGripper::commandThreadFunction, this);
}

Ag2f90dGripper::~Ag2f90dGripper() {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        thread_running_ = false;
    }
    queue_cv_.notify_all();

    if (command_thread_.joinable()) {
        command_thread_.join();
    }
}

bool Ag2f90dGripper::initialize(rokae::xMateErProRobot& robot) {
    robot_ptr_ = &robot;

    if (config_.configure_xpanel) {
        std::error_code ec;
        robot.setxPanelRS485(rokae::xPanelOpt::Vout::supply24v, true, ec);
        if (ec) {
            RCLCPP_ERROR(logger_,
                         "Failed to enable Rokae xPanel 24V/RS485 for %s: %s",
                         config_.model_name.c_str(),
                         ec.message().c_str());
            return false;
        }
        RCLCPP_INFO(logger_, "Rokae xPanel 24V/RS485 enabled for %s", config_.model_name.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    int current_pos = -1;
    if (gripGetPosition(robot, current_pos)) {
        position_ = registerToNormalized(current_pos);
        position_command_ = position_;
        last_command_ = position_;
        RCLCPP_INFO(logger_,
                    "%s initial position: register=%d, normalized=%.6f",
                    config_.model_name.c_str(),
                    current_pos,
                    position_);
    } else {
        position_ = 0.0;
        position_command_ = 0.0;
        last_command_ = 0.0;
        RCLCPP_WARN(logger_,
                    "Failed to read %s initial position, using normalized position 0.0",
                    config_.model_name.c_str());
    }

    if (!gripInit(robot)) {
        return false;
    }

    initialized_ = true;
    return true;
}

bool Ag2f90dGripper::readStatusPeriodic() {
    return initialized_;
}

bool Ag2f90dGripper::writeCommandPeriodic() {
    if (!initialized_) {
        return false;
    }

    const double threshold = 0.01;
    if (std::abs(position_command_ - last_command_) <= threshold) {
        return false;
    }

    const int pos_set = normalizedToRegister(position_command_);
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        while (!command_queue_.empty()) {
            command_queue_.pop();
        }
        command_queue_.push({pos_set});
    }
    queue_cv_.notify_one();

    last_command_ = position_command_;
    RCLCPP_INFO(logger_,
                "%s command queued: %.6f -> register %d",
                config_.model_name.c_str(),
                position_command_,
                pos_set);
    return true;
}

bool Ag2f90dGripper::gripInit(rokae::xMateErProRobot& robot) {
    std::error_code ec;
    std::vector<int> init_set = {POWER_ON};
    robot.XPRWModbusRTUReg(config_.slave_id,
                           WRITE_SINGLE_REG,
                           INIT_REG,
                           "int16",
                           1,
                           init_set,
                           false,
                           ec);
    if (ec) {
        RCLCPP_ERROR(logger_, "%s initialization failed: %s",
                     config_.model_name.c_str(), ec.message().c_str());
        return false;
    }

    RCLCPP_INFO(logger_, "%s initialized", config_.model_name.c_str());
    return true;
}

bool Ag2f90dGripper::gripMove(rokae::xMateErProRobot& robot, int pos_set) {
    std::error_code ec;
    pos_set = std::clamp(pos_set, OPEN_POS, config_.close_pos);

    std::vector<int> pos_set_vec = {0x0000, pos_set};
    robot.XPRWModbusRTUReg(config_.slave_id,
                           WRITE_MULTI_REGS,
                           POS_REG,
                           "int16",
                           2,
                           pos_set_vec,
                           false,
                           ec);
    if (ec) {
        RCLCPP_ERROR(logger_,
                     "%s position write failed (register=%d): %s",
                     config_.model_name.c_str(),
                     pos_set,
                     ec.message().c_str());
        return false;
    }

    std::vector<int> trigger_vec = {TRIGGER_MOVE};
    robot.XPRWModbusRTUReg(config_.slave_id,
                           WRITE_SINGLE_REG,
                           TRIGGER_REG,
                           "int16",
                           1,
                           trigger_vec,
                           false,
                           ec);
    if (ec) {
        RCLCPP_ERROR(logger_, "%s trigger write failed: %s",
                     config_.model_name.c_str(), ec.message().c_str());
        return false;
    }

    return true;
}

bool Ag2f90dGripper::gripGetPosition(rokae::xMateErProRobot& robot, int& pos_now_get) {
    std::error_code ec;
    std::vector<int> pos_now_get_vec = {0, 0};
    robot.XPRWModbusRTUReg(config_.slave_id,
                           READ_HOLDING_REGS,
                           config_.feedback_reg,
                           "int16",
                           2,
                           pos_now_get_vec,
                           false,
                           ec);
    if (ec) {
        RCLCPP_WARN_THROTTLE(logger_,
                             *clock_,
                             2000,
                             "%s position read failed: %s",
                             config_.model_name.c_str(),
                             ec.message().c_str());
        return false;
    }

    uint32_t raw_pos = (static_cast<uint32_t>(pos_now_get_vec[0] & 0xFFFF) << 16) |
                       static_cast<uint32_t>(pos_now_get_vec[1] & 0xFFFF);
    if (raw_pos > static_cast<uint32_t>(config_.close_pos)) {
        raw_pos = (static_cast<uint32_t>(pos_now_get_vec[1] & 0xFFFF) << 16) |
                  static_cast<uint32_t>(pos_now_get_vec[0] & 0xFFFF);
    }
    if (raw_pos > static_cast<uint32_t>(config_.close_pos)) {
        raw_pos = static_cast<uint32_t>(config_.close_pos);
    }

    pos_now_get = static_cast<int>(raw_pos);
    return true;
}

void Ag2f90dGripper::readStatusInternal(rokae::xMateErProRobot& robot) {
    int current_pos = -1;
    if (!gripGetPosition(robot, current_pos)) {
        return;
    }

    const double new_position = registerToNormalized(current_pos);
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        position_ = new_position;
    }
}

void Ag2f90dGripper::readStatusInBackground() {
    if (robot_ptr_ == nullptr || !initialized_) {
        return;
    }

    try {
        readStatusInternal(*robot_ptr_);
        RCLCPP_DEBUG_THROTTLE(logger_,
                              *clock_,
                              1000,
                              "%s position updated: %.6f",
                              config_.model_name.c_str(),
                              position_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(logger_,
                              *clock_,
                              2000,
                              "Failed to read %s state: %s",
                              config_.model_name.c_str(),
                              e.what());
    }
}

void Ag2f90dGripper::commandThreadFunction() {
    auto last_read_time = std::chrono::steady_clock::now();

    while (thread_running_) {
        Command cmd{};
        bool has_command = false;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            auto now = std::chrono::steady_clock::now();
            auto time_since_read = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_read_time);
            const auto read_interval = std::chrono::milliseconds(config_.read_interval_ms);

            if (time_since_read < read_interval && command_queue_.empty()) {
                queue_cv_.wait_for(lock,
                                   read_interval - time_since_read,
                                   [this] {
                                       return !command_queue_.empty() || !thread_running_;
                                   });
            }

            if (!thread_running_) {
                break;
            }

            now = std::chrono::steady_clock::now();
            time_since_read = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_read_time);
            if (time_since_read >= read_interval) {
                lock.unlock();
                readStatusInBackground();
                last_read_time = std::chrono::steady_clock::now();
                lock.lock();
            }

            if (!command_queue_.empty()) {
                cmd = command_queue_.front();
                command_queue_.pop();
                has_command = true;
            }
        }

        if (has_command && robot_ptr_ != nullptr && initialized_) {
            try {
                gripMove(*robot_ptr_, cmd.pos_set);
                RCLCPP_INFO(logger_, "%s command sent: register=%d",
                            config_.model_name.c_str(), cmd.pos_set);
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(logger_,
                                      *clock_,
                                      1000,
                                      "Failed to send %s command: %s",
                                      config_.model_name.c_str(),
                                      e.what());
            }
        }
    }
}

int Ag2f90dGripper::normalizedToRegister(double normalized_position) const {
    normalized_position = std::clamp(normalized_position, 0.0, 1.0);
    const double closed_fraction = 1.0 - normalized_position;
    return static_cast<int>(std::lround(closed_fraction * config_.close_pos));
}

double Ag2f90dGripper::registerToNormalized(int register_pos) const {
    register_pos = std::clamp(register_pos, OPEN_POS, config_.close_pos);
    return 1.0 - (static_cast<double>(register_pos) / config_.close_pos);
}

void Ag2f90dGripper::exportStateInterfaces(
    std::vector<hardware_interface::StateInterface::ConstSharedPtr>& state_interfaces) {
    state_interfaces.push_back(
        std::make_shared<hardware_interface::StateInterface>(
            joint_name_, hardware_interface::HW_IF_POSITION, getPositionPtr()));
    state_interfaces.push_back(
        std::make_shared<hardware_interface::StateInterface>(
            joint_name_, hardware_interface::HW_IF_VELOCITY, getVelocityPtr()));
    state_interfaces.push_back(
        std::make_shared<hardware_interface::StateInterface>(
            joint_name_, hardware_interface::HW_IF_EFFORT, getEffortPtr()));
}

void Ag2f90dGripper::exportCommandInterfaces(
    std::vector<hardware_interface::CommandInterface::SharedPtr>& command_interfaces) {
    command_interfaces.push_back(
        std::make_shared<hardware_interface::CommandInterface>(
            joint_name_, hardware_interface::HW_IF_POSITION, getPositionCommandPtr()));
}

} // namespace rokae_ros2_control
