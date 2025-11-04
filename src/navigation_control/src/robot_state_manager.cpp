/**
 * @file robot_state_manager.cpp
 * @brief 机器人状态管理器 - 统一管理整个系统的状态
 * 
 * 功能:
 * 1. 监控所有子系统状态 (建图、定位、导航、串口)
 * 2. 提供系统模式切换 (建图模式 <-> 导航模式)
 * 3. 发布综合状态信息
 * 4. 提供紧急停止功能
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

enum class SystemMode {
    IDLE,
    MAPPING,
    NAVIGATION
};

class RobotStateManager : public rclcpp::Node
{
public:
    RobotStateManager() : Node("robot_state_manager")
    {
        // 状态订阅器
        mapping_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "mapping_status", 10,
            std::bind(&RobotStateManager::mappingStatusCallback, this, std::placeholders::_1));
        
        localization_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "localization_status", 10,
            std::bind(&RobotStateManager::localizationStatusCallback, this, std::placeholders::_1));
        
        navigation_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "navigation_status", 10,
            std::bind(&RobotStateManager::navigationStatusCallback, this, std::placeholders::_1));
        
        serial_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "serial_status", 10,
            std::bind(&RobotStateManager::serialStatusCallback, this, std::placeholders::_1));
        
        // 综合状态发布器
        system_status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "system_status", 10);
        
        // 紧急停止发布器
        emergency_stop_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        // 服务客户端 - 控制各个子系统
        start_mapping_client_ = this->create_client<std_srvs::srv::Trigger>("start_mapping");
        stop_mapping_client_ = this->create_client<std_srvs::srv::Trigger>("stop_mapping");
        save_map_client_ = this->create_client<std_srvs::srv::Trigger>("save_map");
        
        start_localization_client_ = this->create_client<std_srvs::srv::Trigger>("start_localization");
        stop_localization_client_ = this->create_client<std_srvs::srv::Trigger>("stop_localization");
        
        navigate_to_goal_client_ = this->create_client<std_srvs::srv::Trigger>("navigate_to_goal");
        cancel_navigation_client_ = this->create_client<std_srvs::srv::Trigger>("cancel_navigation");
        
        // 服务服务器 - 模式切换
        switch_to_mapping_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "switch_to_mapping",
            std::bind(&RobotStateManager::switchToMappingCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        switch_to_navigation_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "switch_to_navigation",
            std::bind(&RobotStateManager::switchToNavigationCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        emergency_stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "emergency_stop",
            std::bind(&RobotStateManager::emergencyStopCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // 定时器 - 发布系统状态
        status_timer_ = this->create_wall_timer(
            1s, std::bind(&RobotStateManager::publishSystemStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Robot State Manager initialized");
        RCLCPP_INFO(this->get_logger(), "Current mode: IDLE");
    }

private:
    // 状态回调函数
    void mappingStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        mapping_status_ = msg->data;
    }
    
    void localizationStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        localization_status_ = msg->data;
    }
    
    void navigationStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        navigation_status_ = msg->data;
    }
    
    void serialStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        serial_status_ = msg->data;
    }
    
    // 切换到建图模式
    void switchToMappingCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        RCLCPP_INFO(this->get_logger(), "Switching to MAPPING mode...");
        
        // 停止定位和导航
        if (current_mode_ == SystemMode::NAVIGATION) {
            callService(stop_localization_client_, "stop_localization");
            callService(cancel_navigation_client_, "cancel_navigation");
        }
        
        // 启动建图
        if (callService(start_mapping_client_, "start_mapping")) {
            current_mode_ = SystemMode::MAPPING;
            response->success = true;
            response->message = "Switched to MAPPING mode";
            RCLCPP_INFO(this->get_logger(), "Switched to MAPPING mode successfully");
        } else {
            response->success = false;
            response->message = "Failed to start mapping";
            RCLCPP_ERROR(this->get_logger(), "Failed to switch to MAPPING mode");
        }
    }
    
    // 切换到导航模式
    void switchToNavigationCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        RCLCPP_INFO(this->get_logger(), "Switching to NAVIGATION mode...");
        
        // 停止建图
        if (current_mode_ == SystemMode::MAPPING) {
            callService(stop_mapping_client_, "stop_mapping");
            
            // 保存地图
            RCLCPP_INFO(this->get_logger(), "Saving map before switching...");
            callService(save_map_client_, "save_map");
        }
        
        // 启动定位
        if (callService(start_localization_client_, "start_localization")) {
            current_mode_ = SystemMode::NAVIGATION;
            response->success = true;
            response->message = "Switched to NAVIGATION mode";
            RCLCPP_INFO(this->get_logger(), "Switched to NAVIGATION mode successfully");
        } else {
            response->success = false;
            response->message = "Failed to start localization";
            RCLCPP_ERROR(this->get_logger(), "Failed to switch to NAVIGATION mode");
        }
    }
    
    // 紧急停止
    void emergencyStopCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP activated!");
        
        // 发送停止命令
        auto stop_msg = geometry_msgs::msg::Twist();
        for (int i = 0; i < 10; ++i) {
            emergency_stop_pub_->publish(stop_msg);
            rclcpp::sleep_for(10ms);
        }
        
        // 取消导航
        if (current_mode_ == SystemMode::NAVIGATION) {
            callService(cancel_navigation_client_, "cancel_navigation");
        }
        
        is_emergency_stopped_ = true;
        
        response->success = true;
        response->message = "Emergency stop executed";
    }
    
    // 调用服务的辅助函数
    bool callService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client, 
                     const std::string& service_name)
    {
        if (!client->wait_for_service(2s)) {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available", service_name.c_str());
            return false;
        }
        
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = client->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, 5s) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            if (result.get()->success) {
                RCLCPP_INFO(this->get_logger(), "%s: %s", 
                           service_name.c_str(), result.get()->message.c_str());
                return true;
            } else {
                RCLCPP_WARN(this->get_logger(), "%s failed: %s", 
                           service_name.c_str(), result.get()->message.c_str());
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service %s", service_name.c_str());
            return false;
        }
    }
    
    // 发布系统状态
    void publishSystemStatus()
    {
        auto msg = std_msgs::msg::String();
        
        std::string mode_str;
        switch (current_mode_) {
            case SystemMode::IDLE:
                mode_str = "IDLE";
                break;
            case SystemMode::MAPPING:
                mode_str = "MAPPING";
                break;
            case SystemMode::NAVIGATION:
                mode_str = "NAVIGATION";
                break;
        }
        
        msg.data = "=== SYSTEM STATUS ===\n";
        msg.data += "Mode: " + mode_str + "\n";
        msg.data += "Emergency Stop: " + std::string(is_emergency_stopped_ ? "YES" : "NO") + "\n";
        msg.data += "Mapping: " + mapping_status_ + "\n";
        msg.data += "Localization: " + localization_status_ + "\n";
        msg.data += "Navigation: " + navigation_status_ + "\n";
        msg.data += "Serial: " + serial_status_;
        
        system_status_pub_->publish(msg);
    }
    
    // 成员变量
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapping_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr serial_status_sub_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr emergency_stop_pub_;
    
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_mapping_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_mapping_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_map_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_localization_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_localization_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr navigate_to_goal_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_navigation_client_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr switch_to_mapping_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr switch_to_navigation_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
    
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    SystemMode current_mode_{SystemMode::IDLE};
    bool is_emergency_stopped_{false};
    
    std::string mapping_status_{"Unknown"};
    std::string localization_status_{"Unknown"};
    std::string navigation_status_{"Unknown"};
    std::string serial_status_{"Unknown"};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStateManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
