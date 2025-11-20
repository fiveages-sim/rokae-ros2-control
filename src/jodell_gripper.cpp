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

JodellGripper::JodellGripper(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, const std::string& joint_name)
    : logger_(std::move(logger))
    , clock_(std::move(clock))
    , joint_name_(joint_name)
{
    // 启动后台命令处理线程
    thread_running_ = true;
    command_thread_ = std::thread(&JodellGripper::commandThreadFunction, this);
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
    // 保存机器人对象指针，供后台线程使用
    robot_ptr_ = &robot;

    // 等待初始化完成
    while (true) {
        // 获取初始化状态
        uint8_t init_status = 0;
        bool init_finished = gripGetInitStatus(robot, init_status);
        RCLCPP_INFO(logger_, "Gripper initialize status: %d", init_status);
        if (init_finished) {
            RCLCPP_INFO(logger_, "Gripper initialize success");
            break;
        }
        // 读取一次数据初始化
        std::vector<int> robot_stats;
        gripGetStatus(robot, robot_stats);
        const int pos_now_get = robot_stats[1] >> 8;
        position_ = 1.0 - round(pos_now_get / 255.0);
        physical_position_ = position_ * MAX_OPENING_DISTANCE;
        last_command_ = physical_position_;  // 初始化时设置上一次命令为当前物理位置
        stopped_ = (robot_stats[0] >> 3 & 1) == 0;
    }

    // 执行初始化命令
    gripInit(robot);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));  // 5s用于检查初始化，Jodell电爪初始化需要7s左右

    initialized_ = true;
    return true;
}

bool JodellGripper::readStatusPeriodic() {
    if (!initialized_) {
        return false;
    }

    // 读取操作已在后台线程中异步执行，这里只返回成功
    // 状态数据通过 status_mutex_ 保护，后台线程会定期更新
    // 主线程可以安全地读取 physical_position_ 等状态（已通过互斥锁保护）
    return true;
}

bool JodellGripper::writeCommandPeriodic() {
    if (!initialized_) {
        return false;
    }

    // 只在位置变化超过阈值时发送命令（避免频繁发送）
    // 阈值使用物理位置（米），约0.0004米（约1%的最大张开距离）
    const double threshold = MAX_OPENING_DISTANCE * 0.01;
    if (std::abs(position_command_ - last_command_) > threshold) {
        // 将物理位置（米）转换为归一化值（0.0-1.0）
        double normalized_command = position_command_ / MAX_OPENING_DISTANCE;
        // 限制在有效范围内
        normalized_command = std::max(0.0, std::min(1.0, normalized_command));
        
        RCLCPP_DEBUG(logger_, 
                   "Gripper position command changed: %.6f m -> %.6f m (current: %.6f m, normalized: %.3f)", 
                   last_command_, position_command_, physical_position_, normalized_command);
        
        int pos_set = 255 - static_cast<int>(normalized_command * 255);
        int trq_set = 255;
        int vel_set = 255;
        
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
        
        RCLCPP_DEBUG(logger_, "Gripper command queued: %d (physical: %.6f m, normalized: %.3f)", 
                    pos_set, position_command_, normalized_command);
        return true;
    }
    
    return false;
}

void JodellGripper::gripInit(rokae::xMateErProRobot& robot) {
    std::error_code ec;
    
    // 第一步：写入 0x00 初始化
    int num_write = 1;
    std::vector<int> init_set = {0x00};
    robot.XPRWModbusRTUReg(SLAVE_ADDRESS, INIT_FUNC_CODE, INIT_REG_ADDR, "int16", 
                           num_write, init_set, false, ec);
    RCLCPP_INFO(logger_, "Jodell初始化执行结果 1: %d, %d", ec.value(), init_set[0]);
    
    // 第二步：写入 0x01 启动初始化
    std::vector<int> init_set_up = {0x01};
    robot.XPRWModbusRTUReg(SLAVE_ADDRESS, INIT_FUNC_CODE, INIT_REG_ADDR, "int16", 
                           num_write, init_set_up, false, ec);
    RCLCPP_INFO(logger_, "Jodell初始化执行结果 2: %d, %d", ec.value(), init_set_up[0]);
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
}

bool JodellGripper::gripGetInitStatus(rokae::xMateErProRobot& robot, uint8_t& init_get) {
    std::error_code ec;
    int num_read = 1;
    std::vector init_get_vec = {-1};
    
    robot.XPRWModbusRTUReg(SLAVE_ADDRESS, STATUS_FUNC_CODE, STATUS_REG_ADDR, "int16", 
                           num_read, init_get_vec, false, ec);
    init_get = init_get_vec[0];
    
    int bit4 = init_get >> 4 & 1;
    int bit5 = init_get >> 5 & 1;
    
    // 合并为整数：bit5 作为高位，bit4 作为低位
    int result = (bit5 << 1) | bit4;
    RCLCPP_INFO(logger_, "获取初始化状态执行结果: %d", ec.value());
    
    return result == 3;
}

void JodellGripper::gripGetStatus(rokae::xMateErProRobot& robot, std::vector<int>& robot_stats) {
    std::error_code ec;
    int num_read = 3;
    robot_stats = {-1, -1, -1};
    
    robot.XPRWModbusRTUReg(SLAVE_ADDRESS, STATUS_FUNC_CODE, STATUS_REG_ADDR, "int16", 
                           num_read, robot_stats, false, ec);
}

void JodellGripper::readStatusInternal(rokae::xMateErProRobot& robot) {
    std::vector<int> robot_stats;
    gripGetStatus(robot, robot_stats);
    int pos_now_get = robot_stats[1] >> 8;
    double new_position = 1.0 - round(pos_now_get / 255.0);
    // 将归一化位置映射到物理位置（米）
    double new_physical_position = new_position * MAX_OPENING_DISTANCE;
    bool new_stopped = (robot_stats[0] >> 3 & 1) == 0;
    
    // 使用互斥锁保护状态更新
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        position_ = new_position;
        physical_position_ = new_physical_position;
        stopped_ = new_stopped;
    }
}

void JodellGripper::readStatusInBackground() {
    if (robot_ptr_ == nullptr || !initialized_) {
        return;
    }
    
    try {
        readStatusInternal(*robot_ptr_);
        RCLCPP_DEBUG_THROTTLE(
            logger_,
            *clock_,
            1000,
            "Gripper position updated in background: %.6f m (normalized: %.3f)", 
            physical_position_, position_
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
    auto last_read_time = std::chrono::steady_clock::now();
    
    while (thread_running_) {
        Command cmd;
        bool has_command = false;
        
        // 等待队列中有命令或线程停止信号，或到达读取时间
        {
            std::unique_lock lock(queue_mutex_);
            auto now = std::chrono::steady_clock::now();
            auto time_since_read = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_read_time).count();
            
            // 如果到达读取时间或队列有命令，则处理
            bool should_read = time_since_read >= READ_INTERVAL_MS;
            bool has_queue_command = !command_queue_.empty();
            
            if (!should_read && !has_queue_command) {
                // 等待到下一个读取时间或队列有命令
                auto wait_time = std::chrono::milliseconds(READ_INTERVAL_MS - time_since_read);
                queue_cv_.wait_for(lock, wait_time, [this] { 
                    return !command_queue_.empty() || !thread_running_; 
                });
            }
            
            if (!thread_running_) {
                break;
            }
            
            // 检查是否需要读取状态
            now = std::chrono::steady_clock::now();
            time_since_read = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_read_time).count();
            
            if (time_since_read >= READ_INTERVAL_MS) {
                // 在锁外执行读取，避免阻塞队列操作
                lock.unlock();
                readStatusInBackground();
                last_read_time = std::chrono::steady_clock::now();
                lock.lock();
            }
            
            // 处理队列中的命令
            if (!command_queue_.empty()) {
                cmd = command_queue_.front();
                command_queue_.pop();
                has_command = true;
            }
        }
        
        // 在锁外执行命令，避免阻塞队列操作
        if (has_command && robot_ptr_ != nullptr && initialized_) {
            try {
                // 发送两次命令以提高可靠性
                sendCommand(*robot_ptr_, cmd.pos_set, cmd.trq_set, cmd.vel_set);
                sendCommand(*robot_ptr_, cmd.pos_set, cmd.trq_set, cmd.vel_set);
                
                RCLCPP_DEBUG(logger_, "Gripper command sent from queue: pos=%d, trq=%d, vel=%d", 
                            cmd.pos_set, cmd.trq_set, cmd.vel_set);
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(
                    logger_,
                    *clock_,
                    1000,
                    "Failed to send gripper command from queue: %s", e.what()
                );
            }
        }
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

