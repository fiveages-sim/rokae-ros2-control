#include "rokae_ros2_control/jodell_gripper.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <thread>
#include <chrono>
#include <system_error>
#include <utility>

namespace rokae_ros2_control {

std::string JodellGripper::detectGripperJoint(const std::vector<hardware_interface::ComponentInfo>& joints) {
    for (const auto& joint : joints) {
        // 检查关节名称中是否包含 gripper 或 hand
        std::string joint_name_lower = joint.name;
        std::transform(joint_name_lower.begin(), joint_name_lower.end(), 
                      joint_name_lower.begin(), tolower);
        
        if (joint_name_lower.find("gripper") != std::string::npos || 
            joint_name_lower.find("hand") != std::string::npos) {
            // 这是夹爪关节
            return joint.name;
        }
    }
    
    return "";  // 未检测到夹爪
}

JodellGripper::JodellGripper(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    const std::string& joint_name,
    double normalized_torque,
    double normalized_velocity)
    : logger_(std::move(logger))
    , clock_(std::move(clock))
    , joint_name_(joint_name)
    , command_torque_(static_cast<int>(
          std::clamp(normalized_torque, 0.0, 1.0) * 255.0))
    , command_velocity_(static_cast<int>(
          std::clamp(normalized_velocity, 0.0, 1.0) * 255.0))
{
    // 启动后台命令处理线程
    thread_running_ = true;
    command_thread_ = std::thread(&JodellGripper::commandThreadFunction, this);
}

void JodellGripper::setToolDynamics(double normalized_torque, double normalized_velocity) {
    const int next_torque = static_cast<int>(std::clamp(normalized_torque, 0.0, 1.0) * 255.0);
    const int next_velocity = static_cast<int>(std::clamp(normalized_velocity, 0.0, 1.0) * 255.0);
    std::lock_guard<std::mutex> lock(dynamics_mutex_);
    if (next_torque == command_torque_ && next_velocity == command_velocity_) {
        return;
    }

    command_torque_ = next_torque;
    command_velocity_ = next_velocity;
    dynamics_dirty_ = true;
}

JodellGripper::~JodellGripper() {
    // 停止后台线程
    {
        std::lock_guard lock(queue_mutex_);
        thread_running_ = false;
    }
    queue_cv_.notify_all();
    
    // 等待线程结束
    if (command_thread_.joinable()) {
        command_thread_.join();
    }
}


bool JodellGripper::initialize(rokae::xMateErProRobot& robot) {
    using Clock = std::chrono::steady_clock;

    // 保存机器人对象指针，供后台线程使用
    robot_ptr_ = &robot;

    uint8_t init_status = 0;
    bool init_finished = false;
    bool init_status_read = false;
    auto last_transaction_start = Clock::now();
    for (int attempt = 1; attempt <= INIT_STATUS_READ_ATTEMPTS; ++attempt) {
        last_transaction_start = Clock::now();
        if (gripGetInitStatus(robot, init_status, init_finished)) {
            init_status_read = true;
            break;
        }
        if (attempt < INIT_STATUS_READ_ATTEMPTS) {
            std::this_thread::sleep_until(
                last_transaction_start + COMMUNICATION_INTERVAL);
        }
    }

    if (!init_status_read) {
        RCLCPP_ERROR(logger_,
                     "Unable to determine Jodell initialization state after %d attempts; "
                     "initialization command will not be sent",
                     INIT_STATUS_READ_ATTEMPTS);
        return false;
    }

    if (init_finished) {
        RCLCPP_INFO(logger_, "Jodell gripper is already initialized");
    }

    if (!init_finished) {
        std::this_thread::sleep_until(last_transaction_start + COMMUNICATION_INTERVAL);
        if (!gripInit(robot)) {
            return false;
        }

        const auto deadline =
            Clock::now() + std::chrono::milliseconds(INIT_TIMEOUT_MS);
        uint8_t previous_status = 0xff;
        while (Clock::now() < deadline) {
            last_transaction_start = Clock::now();
            if (gripGetInitStatus(robot, init_status, init_finished)) {
                if (init_status != previous_status) {
                    RCLCPP_INFO(logger_, "Jodell initialize status: %u", init_status);
                    previous_status = init_status;
                }
                if (init_finished) {
                    std::this_thread::sleep_until(
                        last_transaction_start + COMMUNICATION_INTERVAL);
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(INIT_POLL_INTERVAL_MS));
        }

        if (!init_finished) {
            RCLCPP_ERROR(logger_, "Jodell initialization timed out after %d ms", INIT_TIMEOUT_MS);
            return false;
        }
    } else {
        std::this_thread::sleep_until(last_transaction_start + COMMUNICATION_INTERVAL);
    }

    // 这里只读取实际位置来同步ROS状态和命令基准，不发送初始位置命令。
    std::vector<int> robot_stats;
    last_transaction_start = Clock::now();
    if (gripGetStatus(robot, robot_stats)) {
        updateStatusFromRegisters(robot_stats);
        copyStatusSnapshot();
        position_command_ = physical_position_;
        last_command_ = physical_position_;
    } else {
        RCLCPP_WARN(logger_,
                    "Unable to read Jodell position after initialization; preserving default state");
    }

    // 防止后台线程在最终状态读取后立即发起下一笔事务。
    std::this_thread::sleep_until(last_transaction_start + COMMUNICATION_INTERVAL);
    initialized_.store(true, std::memory_order_release);
    RCLCPP_INFO(logger_, "Jodell gripper initialization succeeded");
    return true;
}

bool JodellGripper::readStatusPeriodic() {
    if (!initialized_) {
        return false;
    }

    // Modbus读取由后台线程执行；ros2_control线程只复制完整快照，避免
    // controller_manager与通信线程同时访问导出的状态变量。
    return copyStatusSnapshot();
}

bool JodellGripper::writeCommandPeriodic() {
    if (!initialized_) {
        return false;
    }

    // 只在位置变化超过阈值时发送命令（避免频繁发送）
    // 阈值使用物理位置（米），约0.0004米（约1%的最大张开距离）
    const double threshold = MAX_OPENING_DISTANCE * 0.01;
    int trq_set = 0;
    int vel_set = 0;
    bool dynamics_dirty = false;
    {
        std::lock_guard<std::mutex> lock(dynamics_mutex_);
        trq_set = command_torque_;
        vel_set = command_velocity_;
        dynamics_dirty = dynamics_dirty_;
    }

    if (std::abs(position_command_ - last_command_) > threshold || dynamics_dirty) {
        // 将物理位置（米）转换为归一化值（0.0-1.0）
        double normalized_command = position_command_ / MAX_OPENING_DISTANCE;
        // 限制在有效范围内
        normalized_command = std::max(0.0, std::min(1.0, normalized_command));
        
        RCLCPP_DEBUG(logger_, 
                   "Gripper position command changed: %.6f m -> %.6f m (current: %.6f m, normalized: %.3f)", 
                   last_command_, position_command_, physical_position_, normalized_command);
        
        int pos_set = 255 - static_cast<int>(normalized_command * 255);
        // 将命令加入队列，由后台线程处理（非阻塞）
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            // 清空旧命令，只保留最新的命令（避免队列堆积）
            while (!command_queue_.empty()) {
                command_queue_.pop();
            }
            command_queue_.push({pos_set, trq_set, vel_set});
        }
        queue_cv_.notify_one();
        
        stopped_ = false;
        last_command_ = position_command_;
        {
            std::lock_guard<std::mutex> lock(dynamics_mutex_);
            dynamics_dirty_ = false;
        }
        
        RCLCPP_INFO(logger_, "Gripper command queued: pos=%d, force=%d, vel=%d (physical: %.6f m, normalized: %.3f)",
                    pos_set, trq_set, vel_set, position_command_, normalized_command);
        return true;
    }
    
    return false;
}

bool JodellGripper::gripInit(rokae::xMateErProRobot& robot) {
    using Clock = std::chrono::steady_clock;
    std::error_code ec;
    
    // 第一步：写入 0x00 初始化
    int num_write = 1;
    std::vector<int> init_set = {0x00};
    const auto reset_start = Clock::now();
    robot.XPRWModbusRTUReg(SLAVE_ADDRESS, INIT_FUNC_CODE, INIT_REG_ADDR, "int16", 
                           num_write, init_set, false, ec);
    RCLCPP_INFO(logger_, "Jodell初始化执行结果 1: %d, %d", ec.value(), init_set[0]);
    if (ec) {
        RCLCPP_ERROR(logger_, "Failed to reset Jodell initialization: %s", ec.message().c_str());
        return false;
    }

    std::this_thread::sleep_until(reset_start + COMMUNICATION_INTERVAL);

    // 第二步：写入 0x01 启动初始化
    std::vector<int> init_set_up = {0x01};
    const auto activate_start = Clock::now();
    robot.XPRWModbusRTUReg(SLAVE_ADDRESS, INIT_FUNC_CODE, INIT_REG_ADDR, "int16", 
                           num_write, init_set_up, false, ec);
    RCLCPP_INFO(logger_, "Jodell初始化执行结果 2: %d, %d", ec.value(), init_set_up[0]);
    if (ec) {
        RCLCPP_ERROR(logger_, "Failed to start Jodell initialization: %s", ec.message().c_str());
        return false;
    }

    std::this_thread::sleep_until(activate_start + COMMUNICATION_INTERVAL);
    return true;
}

void JodellGripper::gripMove(rokae::xMateErProRobot& robot, int trq_set, int vel_set, int pos_set) {
    std::error_code ec;
    
    // 构建控制数据
    std::vector<int> trq_set_vec;
    uint16_t trigger = 0x09;
    uint16_t position = pos_set << 8;
    uint16_t speed_force = trq_set << 8 | vel_set;
    trq_set_vec.push_back(trigger);
    trq_set_vec.push_back(position);
    trq_set_vec.push_back(speed_force);

    int num_write = 3;
    robot.XPRWModbusRTUReg(SLAVE_ADDRESS, INIT_FUNC_CODE, INIT_REG_ADDR, "int16", 
                           num_write, trq_set_vec, false, ec);
    if (ec) {
        // RG75在启动实际运动时可能已经接受命令，但SDK未在超时窗口内
        // 收到正常应答。后续FC04状态读取负责确认实际执行结果。
        RCLCPP_DEBUG_THROTTLE(
            logger_, *clock_, 1000,
            "Jodell movement command was not acknowledged normally; "
            "execution will be checked by status reads: %s",
            ec.message().c_str());
    }
}

bool JodellGripper::gripGetInitStatus(
    rokae::xMateErProRobot& robot, uint8_t& init_get, bool& init_finished) {
    std::error_code ec;
    int num_read = 1;
    std::vector init_get_vec = {-1};
    
    robot.XPRWModbusRTUReg(SLAVE_ADDRESS, STATUS_FUNC_CODE, STATUS_REG_ADDR, "int16", 
                           num_read, init_get_vec, false, ec);
    if (ec) {
        init_finished = false;
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                             "Failed to read Jodell initialization status: %s",
                             ec.message().c_str());
        return false;
    }

    init_get = static_cast<uint8_t>(init_get_vec[0] & 0xff);
    
    int bit4 = init_get >> 4 & 1;
    int bit5 = init_get >> 5 & 1;
    
    // 合并为整数：bit5 作为高位，bit4 作为低位
    int result = (bit5 << 1) | bit4;
    init_finished = (result == 3);
    return true;
}

bool JodellGripper::gripGetStatus(rokae::xMateErProRobot& robot, std::vector<int>& robot_stats) {
    std::error_code ec;
    int num_read = 3;
    robot_stats = {-1, -1, -1};
    
    robot.XPRWModbusRTUReg(SLAVE_ADDRESS, STATUS_FUNC_CODE, STATUS_REG_ADDR, "int16", 
                           num_read, robot_stats, false, ec);
    if (ec) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                             "Failed to read Jodell state: %s", ec.message().c_str());
        return false;
    }
    return true;
}

void JodellGripper::updateStatusFromRegisters(const std::vector<int>& robot_stats) {
    if (robot_stats.size() < 3) {
        return;
    }

    const uint8_t new_status_byte = static_cast<uint8_t>(robot_stats[0] & 0xff);
    const int pos_now_get = (robot_stats[1] >> 8) & 0xff;
    const uint8_t new_error_code = static_cast<uint8_t>(robot_stats[1] & 0xff);
    const int new_velocity = robot_stats[2] & 0xff;
    const int raw_force = (robot_stats[2] >> 8) & 0xff;
    const double new_position = 1.0 - (static_cast<double>(pos_now_get) / 255.0);
    const double new_physical_position = new_position * MAX_OPENING_DISTANCE;
    const bool new_stopped = ((new_status_byte >> 3) & 1) == 0;
    const uint8_t object_status = (new_status_byte >> 6) & 0x03;
    const bool object_detected = object_status == 0x01 || object_status == 0x02;
    
    StatusSnapshot snapshot;
    snapshot.position = new_position;
    snapshot.physical_position = new_physical_position;
    snapshot.velocity = static_cast<double>(new_velocity);
    // RG75 register 2 high byte is the configured/raw force byte, not a reliable
    // continuous contact force. Expose effort as a contact flag decoded from
    // the gripper object-status bits so adaptive force feedback only triggers
    // on actual object detection.
    snapshot.effort = object_detected ? 1.0 : 0.0;
    snapshot.stopped = new_stopped;
    snapshot.status_byte = new_status_byte;
    snapshot.error_code = new_error_code;

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        background_status_ = snapshot;
        background_status_valid_ = true;
    }
    (void)raw_force;
}

bool JodellGripper::copyStatusSnapshot() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (!background_status_valid_) {
        return false;
    }

    position_ = background_status_.position;
    physical_position_ = background_status_.physical_position;
    velocity_ = background_status_.velocity;
    effort_ = background_status_.effort;
    stopped_ = background_status_.stopped;
    status_byte_ = background_status_.status_byte;
    error_code_ = background_status_.error_code;
    return true;
}

bool JodellGripper::readStatusInternal(rokae::xMateErProRobot& robot) {
    std::vector<int> robot_stats;
    if (!gripGetStatus(robot, robot_stats)) {
        return false;
    }
    updateStatusFromRegisters(robot_stats);
    return true;
}

void JodellGripper::readStatusInBackground() {
    if (!initialized_.load(std::memory_order_acquire) || robot_ptr_ == nullptr) {
        return;
    }
    
    try {
        if (!readStatusInternal(*robot_ptr_)) {
            return;
        }
        StatusSnapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            if (!background_status_valid_) {
                return;
            }
            snapshot = background_status_;
        }
        RCLCPP_DEBUG_THROTTLE(
            logger_,
            *clock_,
            1000,
            "Gripper state: position=%.6f m (normalized=%.3f), force=%.0f, velocity=%.0f, status=0x%02x, error=0x%02x",
            snapshot.physical_position,
            snapshot.position,
            snapshot.effort,
            snapshot.velocity,
            snapshot.status_byte,
            snapshot.error_code
        );
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(
            logger_,
            *clock_,
            2000,
            "Failed to read gripper state in background: %s", e.what()
        );
    }
}

void JodellGripper::sendCommand(rokae::xMateErProRobot& robot, int pos_set, int trq_set, int vel_set) {
    gripMove(robot, trq_set, vel_set, pos_set);
}

void JodellGripper::commandThreadFunction() {
    using Clock = std::chrono::steady_clock;
    auto next_transaction_time = Clock::now();
    
    while (thread_running_) {
        Command cmd;
        bool has_command = false;

        // 所有运行期Modbus事务按15Hz限频。新命令可以唤醒线程，但不能
        // 绕过下一个事务槽，以保证相邻事务的开始时间至少间隔66.667ms。
        {
            std::unique_lock lock(queue_mutex_);
            while (thread_running_ && Clock::now() < next_transaction_time) {
                queue_cv_.wait_until(lock, next_transaction_time);
            }

            if (!thread_running_) {
                break;
            }

            // 命令优先；没有新命令时，本事务槽用于读取状态。
            if (!command_queue_.empty()) {
                cmd = command_queue_.front();
                command_queue_.pop();
                has_command = true;
            }
        }

        const auto transaction_start = Clock::now();
        if (initialized_.load(std::memory_order_acquire) && robot_ptr_ != nullptr) {
            try {
                if (has_command) {
                    sendCommand(*robot_ptr_, cmd.pos_set, cmd.trq_set, cmd.vel_set);
                    RCLCPP_DEBUG(logger_,
                                 "Gripper command sent from queue: pos=%d, trq=%d, vel=%d",
                                 cmd.pos_set, cmd.trq_set, cmd.vel_set);
                } else {
                    readStatusInBackground();
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(
                    logger_,
                    *clock_,
                    1000,
                    "Failed to send gripper command from queue: %s", e.what()
                );
            }
        }

        next_transaction_time = transaction_start + COMMUNICATION_INTERVAL;
    }
}

void JodellGripper::exportStateInterfaces(std::vector<hardware_interface::StateInterface::ConstSharedPtr>& state_interfaces) {
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

void JodellGripper::exportCommandInterfaces(std::vector<hardware_interface::CommandInterface::SharedPtr>& command_interfaces) {
    command_interfaces.push_back(
        std::make_shared<hardware_interface::CommandInterface>(
            joint_name_, hardware_interface::HW_IF_POSITION, getPositionCommandPtr()));
}

} // namespace rokae_ros2_control
