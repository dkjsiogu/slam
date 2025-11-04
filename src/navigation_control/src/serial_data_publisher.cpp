/**
 * @file serial_data_publisher.cpp
 * @brief 串口数据发布器 - 准备发送给下位机的控制数据
 * 
 * 功能:
 * 1. 订阅Nav2的速度命令 (/cmd_vel)
 * 2. 将速度命令转换为下位机协议格式
 * 3. 发布准备好的串口数据 (待串口模块接入)
 * 4. 提供速度限制和平滑处理
 * 
 * 数据格式准备:
 * - 线速度 (m/s) -> 转换为下位机单位 (mm/s 或 编码器脉冲)
 * - 角速度 (rad/s) -> 转换为下位机单位
 * - 添加校验和、帧头帧尾等
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;

// 串口协议结构体 (示例)
struct SerialProtocol {
    uint8_t header1 = 0xAA;      // 帧头1
    uint8_t header2 = 0x55;      // 帧头2
    uint8_t cmd_id = 0x01;       // 命令ID: 0x01=速度控制
    int16_t linear_vel;          // 线速度 (mm/s)
    int16_t angular_vel;         // 角速度 (mrad/s, 毫弧度/秒)
    uint8_t reserved1 = 0x00;    // 保留字节
    uint8_t reserved2 = 0x00;    // 保留字节
    uint8_t checksum;            // 校验和 (所有字节异或)
    uint8_t tail = 0x0D;         // 帧尾
    
    // 计算校验和
    void calculateChecksum() {
        checksum = header1 ^ header2 ^ cmd_id ^ 
                  (linear_vel & 0xFF) ^ ((linear_vel >> 8) & 0xFF) ^
                  (angular_vel & 0xFF) ^ ((angular_vel >> 8) & 0xFF) ^
                  reserved1 ^ reserved2;
    }
    
    // 转换为字节数组
    std::vector<uint8_t> toBytes() {
        calculateChecksum();
        return {
            header1, header2, cmd_id,
            static_cast<uint8_t>(linear_vel & 0xFF),
            static_cast<uint8_t>((linear_vel >> 8) & 0xFF),
            static_cast<uint8_t>(angular_vel & 0xFF),
            static_cast<uint8_t>((angular_vel >> 8) & 0xFF),
            reserved1, reserved2,
            checksum, tail
        };
    }
};

class SerialDataPublisher : public rclcpp::Node
{
public:
    SerialDataPublisher() : Node("serial_data_publisher")
    {
        // 声明参数
        this->declare_parameter("max_linear_vel", 1.0);      // 最大线速度 m/s
        this->declare_parameter("max_angular_vel", 2.0);     // 最大角速度 rad/s
        this->declare_parameter("linear_scale", 1000.0);     // 线速度缩放 (m/s -> mm/s)
        this->declare_parameter("angular_scale", 1000.0);    // 角速度缩放 (rad/s -> mrad/s)
        this->declare_parameter("velocity_timeout", 1.0);    // 速度命令超时(秒)
        this->declare_parameter("smooth_factor", 0.8);       // 速度平滑因子 (0-1)
        
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        linear_scale_ = this->get_parameter("linear_scale").as_double();
        angular_scale_ = this->get_parameter("angular_scale").as_double();
        velocity_timeout_ = this->get_parameter("velocity_timeout").as_double();
        smooth_factor_ = this->get_parameter("smooth_factor").as_double();
        
        // 订阅速度命令
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SerialDataPublisher::cmdVelCallback, this, std::placeholders::_1));
        
        // 发布原始字节数据 (待串口模块使用)
        serial_data_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "serial_tx_data", 10);
        
        // 发布可读的十六进制字符串 (调试用)
        serial_hex_pub_ = this->create_publisher<std_msgs::msg::String>(
            "serial_tx_hex", 10);
        
        // 发布处理后的速度 (调试用)
        processed_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "processed_cmd_vel", 10);
        
        // 状态发布器
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "serial_status", 10);
        
        // 定时器 - 检查速度命令超时
        timeout_timer_ = this->create_wall_timer(
            100ms, std::bind(&SerialDataPublisher::checkTimeout, this));
        
        // 定时器 - 发布状态
        status_timer_ = this->create_wall_timer(
            1s, std::bind(&SerialDataPublisher::publishStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Serial Data Publisher initialized");
        RCLCPP_INFO(this->get_logger(), "Max linear velocity: %.2f m/s", max_linear_vel_);
        RCLCPP_INFO(this->get_logger(), "Max angular velocity: %.2f rad/s", max_angular_vel_);
        RCLCPP_INFO(this->get_logger(), "Linear scale: %.0f (m/s -> mm/s)", linear_scale_);
        RCLCPP_INFO(this->get_logger(), "Angular scale: %.0f (rad/s -> mrad/s)", angular_scale_);
    }

private:
    // 速度命令回调
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_time_ = this->now();
        
        // 获取原始速度
        double linear = msg->linear.x;
        double angular = msg->angular.z;
        
        // 速度限制
        linear = std::clamp(linear, -max_linear_vel_, max_linear_vel_);
        angular = std::clamp(angular, -max_angular_vel_, max_angular_vel_);
        
        // 速度平滑 (低通滤波)
        smoothed_linear_ = smooth_factor_ * smoothed_linear_ + (1.0 - smooth_factor_) * linear;
        smoothed_angular_ = smooth_factor_ * smoothed_angular_ + (1.0 - smooth_factor_) * angular;
        
        // 发布处理后的速度 (调试用)
        auto processed_msg = geometry_msgs::msg::Twist();
        processed_msg.linear.x = smoothed_linear_;
        processed_msg.angular.z = smoothed_angular_;
        processed_vel_pub_->publish(processed_msg);
        
        // 准备串口数据
        prepareSerialData(smoothed_linear_, smoothed_angular_);
        
        packets_sent_++;
    }
    
    // 准备串口数据
    void prepareSerialData(double linear_vel, double angular_vel)
    {
        SerialProtocol packet;
        
        // 转换为下位机单位
        packet.linear_vel = static_cast<int16_t>(linear_vel * linear_scale_);
        packet.angular_vel = static_cast<int16_t>(angular_vel * angular_scale_);
        
        // 转换为字节数组
        auto bytes = packet.toBytes();
        
        // 发布原始字节数据
        auto data_msg = std_msgs::msg::UInt8MultiArray();
        data_msg.data = bytes;
        serial_data_pub_->publish(data_msg);
        
        // 发布十六进制字符串 (便于调试)
        std::stringstream hex_stream;
        hex_stream << "TX: ";
        for (uint8_t byte : bytes) {
            hex_stream << std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(byte) << " ";
        }
        
        auto hex_msg = std_msgs::msg::String();
        hex_msg.data = hex_stream.str();
        serial_hex_pub_->publish(hex_msg);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Serial data - Linear: %d mm/s, Angular: %d mrad/s, Hex: %s",
                    packet.linear_vel, packet.angular_vel, hex_msg.data.c_str());
    }
    
    // 检查速度命令超时
    void checkTimeout()
    {
        auto now = this->now();
        auto elapsed = (now - last_cmd_time_).seconds();
        
        if (elapsed > velocity_timeout_ && 
            (std::abs(smoothed_linear_) > 0.01 || std::abs(smoothed_angular_) > 0.01)) {
            
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Velocity command timeout (%.1fs) - sending stop command", elapsed);
            
            // 发送停止命令
            smoothed_linear_ = 0.0;
            smoothed_angular_ = 0.0;
            prepareSerialData(0.0, 0.0);
        }
    }
    
    // 发布状态
    void publishStatus()
    {
        auto msg = std_msgs::msg::String();
        
        auto elapsed = (this->now() - last_cmd_time_).seconds();
        
        if (elapsed < velocity_timeout_) {
            msg.data = "ACTIVE - Linear: " + 
                      std::to_string(static_cast<int>(smoothed_linear_ * 1000)) + " mm/s, " +
                      "Angular: " + 
                      std::to_string(static_cast<int>(smoothed_angular_ * 1000)) + " mrad/s, " +
                      "Packets: " + std::to_string(packets_sent_);
        } else {
            msg.data = "IDLE - Total packets sent: " + std::to_string(packets_sent_);
        }
        
        status_pub_->publish(msg);
    }
    
    // 成员变量
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_data_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_hex_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr processed_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    double max_linear_vel_;
    double max_angular_vel_;
    double linear_scale_;
    double angular_scale_;
    double velocity_timeout_;
    double smooth_factor_;
    
    double smoothed_linear_{0.0};
    double smoothed_angular_{0.0};
    
    rclcpp::Time last_cmd_time_;
    int packets_sent_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialDataPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
