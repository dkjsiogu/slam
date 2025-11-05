/**
 * @file omni_wheel_controller.cpp
 * @brief 全向轮控制器 - 将导航速度转换为全向轮控制命令
 * 
 * 功能:
 * 1. 订阅Nav2速度命令 (/cmd_vel)
 * 2. 转换为全向轮运动学参数（Vx, Vy, Wz）
 * 3. 生成带CRC校验的串口协议数据包
 * 4. 发送给下位机
 * 5. 支持手动速度控制（调试用）
 * 
 * 全向轮运动学:
 * - Vx: 前后方向速度 (m/s)
 * - Vy: 左右方向速度 (m/s) - 全向轮特有
 * - Wz: 旋转角速度 (rad/s)
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;

// 全向轮串口协议结构 (15字节)
struct OmniWheelProtocol {
    uint8_t header1 = 0xAA;      // 帧头1
    uint8_t header2 = 0x55;      // 帧头2
    uint8_t cmd_id = 0x10;       // 命令ID: 0x10=全向轮速度控制
    int16_t vx;                  // X方向速度 (mm/s) - 前后
    int16_t vy;                  // Y方向速度 (mm/s) - 左右 (全向轮特有)
    int16_t wz;                  // 旋转角速度 (mrad/s)
    uint8_t mode;                // 控制模式 (0=速度控制, 1=位置控制)
    uint8_t reserved1 = 0x00;    // 保留字节
    uint8_t reserved2 = 0x00;    // 保留字节
    uint16_t crc16;              // CRC-16校验
    uint8_t tail = 0x0D;         // 帧尾
    
    // 计算CRC-16 (MODBUS标准)
    static uint16_t calculateCRC16(const uint8_t* data, size_t length) {
        uint16_t crc = 0xFFFF;
        
        for (size_t i = 0; i < length; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc = crc >> 1;
                }
            }
        }
        
        return crc;
    }
    
    // 转换为字节数组
    std::vector<uint8_t> toBytes() {
        std::vector<uint8_t> bytes = {
            header1, header2, cmd_id,
            static_cast<uint8_t>(vx & 0xFF),
            static_cast<uint8_t>((vx >> 8) & 0xFF),
            static_cast<uint8_t>(vy & 0xFF),
            static_cast<uint8_t>((vy >> 8) & 0xFF),
            static_cast<uint8_t>(wz & 0xFF),
            static_cast<uint8_t>((wz >> 8) & 0xFF),
            mode,
            reserved1,
            reserved2
        };
        
        // 计算CRC (不包括CRC本身和帧尾)
        crc16 = calculateCRC16(bytes.data(), bytes.size());
        
        // 添加CRC和帧尾
        bytes.push_back(static_cast<uint8_t>(crc16 & 0xFF));
        bytes.push_back(static_cast<uint8_t>((crc16 >> 8) & 0xFF));
        bytes.push_back(tail);
        
        return bytes;
    }
};

class OmniWheelController : public rclcpp::Node
{
public:
    OmniWheelController() : Node("omni_wheel_controller")
    {
        // 声明参数
        this->declare_parameter("max_vx", 1.0);              // 最大X速度 m/s
        this->declare_parameter("max_vy", 1.0);              // 最大Y速度 m/s
        this->declare_parameter("max_wz", 2.0);              // 最大旋转速度 rad/s
        this->declare_parameter("velocity_scale", 1000.0);   // 速度缩放 (m/s -> mm/s)
        this->declare_parameter("angular_scale", 1000.0);    // 角速度缩放 (rad/s -> mrad/s)
        this->declare_parameter("velocity_timeout", 1.0);    // 速度命令超时(秒)
        this->declare_parameter("smooth_factor", 0.7);       // 速度平滑因子 (0-1)
        this->declare_parameter("enable_lateral_motion", true); // 启用侧向运动
        
        max_vx_ = this->get_parameter("max_vx").as_double();
        max_vy_ = this->get_parameter("max_vy").as_double();
        max_wz_ = this->get_parameter("max_wz").as_double();
        velocity_scale_ = this->get_parameter("velocity_scale").as_double();
        angular_scale_ = this->get_parameter("angular_scale").as_double();
        velocity_timeout_ = this->get_parameter("velocity_timeout").as_double();
        smooth_factor_ = this->get_parameter("smooth_factor").as_double();
        enable_lateral_motion_ = this->get_parameter("enable_lateral_motion").as_bool();
        
        // 订阅速度命令
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&OmniWheelController::cmdVelCallback, this, std::placeholders::_1));
        
        // 发布串口数据
        serial_tx_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "serial_tx_data", 10);
        
        // 发布十六进制调试信息
        serial_hex_pub_ = this->create_publisher<std_msgs::msg::String>(
            "omni_tx_hex", 10);
        
        // 发布处理后的速度
        processed_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "omni_processed_vel", 10);
        
        // 状态发布
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "omni_status", 10);
        
        // 服务 - 紧急停止
        emergency_stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "emergency_stop",
            std::bind(&OmniWheelController::emergencyStopCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // 定时器 - 检查超时
        timeout_timer_ = this->create_wall_timer(
            100ms, std::bind(&OmniWheelController::checkTimeout, this));
        
        // 定时器 - 发布状态
        status_timer_ = this->create_wall_timer(
            1s, std::bind(&OmniWheelController::publishStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "全向轮控制器已初始化");
        RCLCPP_INFO(this->get_logger(), "最大速度 - Vx: %.2f m/s, Vy: %.2f m/s, Wz: %.2f rad/s",
                   max_vx_, max_vy_, max_wz_);
        RCLCPP_INFO(this->get_logger(), "侧向运动: %s", enable_lateral_motion_ ? "启用" : "禁用");
        RCLCPP_INFO(this->get_logger(), "协议: 15字节 + CRC-16校验");
    }

private:
    // 速度命令回调
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_time_ = this->now();
        is_emergency_stop_ = false;
        
        // 提取速度分量
        double vx = msg->linear.x;   // 前后速度
        double vy = enable_lateral_motion_ ? msg->linear.y : 0.0;  // 左右速度
        double wz = msg->angular.z;  // 旋转速度
        
        // 速度限制
        vx = std::clamp(vx, -max_vx_, max_vx_);
        vy = std::clamp(vy, -max_vy_, max_vy_);
        wz = std::clamp(wz, -max_wz_, max_wz_);
        
        // 速度平滑处理
        smoothed_vx_ = smooth_factor_ * smoothed_vx_ + (1.0 - smooth_factor_) * vx;
        smoothed_vy_ = smooth_factor_ * smoothed_vy_ + (1.0 - smooth_factor_) * vy;
        smoothed_wz_ = smooth_factor_ * smoothed_wz_ + (1.0 - smooth_factor_) * wz;
        
        // 发布处理后的速度
        auto processed_msg = geometry_msgs::msg::Twist();
        processed_msg.linear.x = smoothed_vx_;
        processed_msg.linear.y = smoothed_vy_;
        processed_msg.angular.z = smoothed_wz_;
        processed_vel_pub_->publish(processed_msg);
        
        // 生成并发送串口数据
        sendOmniCommand(smoothed_vx_, smoothed_vy_, smoothed_wz_);
        
        packets_sent_++;
    }
    
    // 发送全向轮控制命令
    void sendOmniCommand(double vx, double vy, double wz)
    {
        OmniWheelProtocol packet;
        
        // 转换为下位机单位
        packet.vx = static_cast<int16_t>(vx * velocity_scale_);
        packet.vy = static_cast<int16_t>(vy * velocity_scale_);
        packet.wz = static_cast<int16_t>(wz * angular_scale_);
        packet.mode = 0;  // 速度控制模式
        
        // 转换为字节数组（包含CRC）
        auto bytes = packet.toBytes();
        
        // 发布原始字节数据
        auto data_msg = std_msgs::msg::UInt8MultiArray();
        data_msg.data = bytes;
        serial_tx_pub_->publish(data_msg);
        
        // 发布十六进制调试信息
        std::stringstream hex_stream;
        hex_stream << "TX [全向轮]: ";
        for (uint8_t byte : bytes) {
            hex_stream << std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(byte) << " ";
        }
        hex_stream << " | Vx=" << packet.vx << "mm/s, Vy=" << packet.vy 
                  << "mm/s, Wz=" << packet.wz << "mrad/s | CRC16=0x" 
                  << std::hex << packet.crc16;
        
        auto hex_msg = std_msgs::msg::String();
        hex_msg.data = hex_stream.str();
        serial_hex_pub_->publish(hex_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "%s", hex_msg.data.c_str());
    }
    
    // 检查速度命令超时
    void checkTimeout()
    {
        if (is_emergency_stop_) {
            return;  // 紧急停止状态，不发送命令
        }
        
        auto now = this->now();
        auto elapsed = (now - last_cmd_time_).seconds();
        
        // 检查是否超时
        if (elapsed > velocity_timeout_) {
            // 检查是否有非零速度
            bool has_velocity = std::abs(smoothed_vx_) > 0.01 || 
                              std::abs(smoothed_vy_) > 0.01 || 
                              std::abs(smoothed_wz_) > 0.01;
            
            if (has_velocity) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                   "速度命令超时 (%.1fs)，发送停止命令", elapsed);
                
                // 逐渐减速停止
                smoothed_vx_ *= 0.5;
                smoothed_vy_ *= 0.5;
                smoothed_wz_ *= 0.5;
                
                // 如果速度已经很小，直接设为0
                if (std::abs(smoothed_vx_) < 0.01) smoothed_vx_ = 0.0;
                if (std::abs(smoothed_vy_) < 0.01) smoothed_vy_ = 0.0;
                if (std::abs(smoothed_wz_) < 0.01) smoothed_wz_ = 0.0;
                
                sendOmniCommand(smoothed_vx_, smoothed_vy_, smoothed_wz_);
            }
        }
    }
    
    // 紧急停止服务
    void emergencyStopCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_WARN(this->get_logger(), "⚠ 紧急停止触发！");
        
        is_emergency_stop_ = true;
        smoothed_vx_ = 0.0;
        smoothed_vy_ = 0.0;
        smoothed_wz_ = 0.0;
        
        // 发送停止命令
        sendOmniCommand(0.0, 0.0, 0.0);
        
        response->success = true;
        response->message = "紧急停止已执行";
    }
    
    // 发布状态信息
    void publishStatus()
    {
        auto status_msg = std_msgs::msg::String();
        
        char buffer[256];
        snprintf(buffer, sizeof(buffer),
                "全向轮控制 | Vx=%.3f m/s, Vy=%.3f m/s, Wz=%.3f rad/s | "
                "已发送=%lu包 | %s",
                smoothed_vx_, smoothed_vy_, smoothed_wz_,
                packets_sent_,
                is_emergency_stop_ ? "紧急停止" : "正常");
        
        status_msg.data = buffer;
        status_pub_->publish(status_msg);
    }
    
    // 成员变量
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_tx_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_hex_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr processed_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
    
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // 参数
    double max_vx_;
    double max_vy_;
    double max_wz_;
    double velocity_scale_;
    double angular_scale_;
    double velocity_timeout_;
    double smooth_factor_;
    bool enable_lateral_motion_;
    
    // 状态变量
    double smoothed_vx_{0.0};
    double smoothed_vy_{0.0};
    double smoothed_wz_{0.0};
    rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
    bool is_emergency_stop_{false};
    unsigned long packets_sent_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OmniWheelController>();
    
    // 初始化时间
    node->get_clock()->now();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
