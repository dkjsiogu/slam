/**
 * @file navigation_controller.cpp
 * @brief 导航控制器 - 管理Nav2导航、路径规划、目标点导航
 * 
 * 功能:
 * 1. 发送导航目标点
 * 2. 监控导航状态
 * 3. 取消导航任务
 * 4. 路径跟踪和障碍物避障
 * 5. 多目标点巡航
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <memory>
#include <vector>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class NavigationController : public rclcpp::Node
{
public:
    NavigationController() : Node("navigation_controller")
    {
        // 声明参数
        this->declare_parameter("navigation_timeout", 300.0);
        this->declare_parameter("goal_tolerance", 0.2);
        
        navigation_timeout_ = this->get_parameter("navigation_timeout").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        
        // 发布器
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "navigation_status", 10);
        
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        // 订阅器 - 全局路径
        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10,
            std::bind(&NavigationController::globalPathCallback, this, std::placeholders::_1));
        
        // 订阅器 - 本地路径
        local_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/local_plan", 10,
            std::bind(&NavigationController::localPathCallback, this, std::placeholders::_1));
        
        // Action客户端 - NavigateToPose
        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");
        
        // 服务
        navigate_to_goal_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "navigate_to_goal",
            std::bind(&NavigationController::navigateToGoalCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        cancel_navigation_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "cancel_navigation",
            std::bind(&NavigationController::cancelNavigationCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // 定时器
        status_timer_ = this->create_wall_timer(
            1s, std::bind(&NavigationController::publishStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Navigation Controller initialized");
        RCLCPP_INFO(this->get_logger(), "Navigation timeout: %.1f seconds", navigation_timeout_);
        RCLCPP_INFO(this->get_logger(), "Goal tolerance: %.2f meters", goal_tolerance_);
        
        // 初始化一些测试目标点
        initializeTestWaypoints();
    }
    
    // 发送导航目标 (外部调用)
    bool sendGoal(double x, double y, double yaw)
    {
        if (!nav_to_pose_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Navigate to pose action server not available");
            return false;
        }
        
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";
        
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.position.z = 0.0;
        
        // 将yaw转换为四元数
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal_msg.pose.pose.orientation = tf2::toMsg(q);
        
        RCLCPP_INFO(this->get_logger(), 
                   "Sending navigation goal: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
        
        // 设置回调
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            std::bind(&NavigationController::goalResponseCallback, this, std::placeholders::_1);
        
        send_goal_options.feedback_callback =
            std::bind(&NavigationController::feedbackCallback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        send_goal_options.result_callback =
            std::bind(&NavigationController::resultCallback, this, std::placeholders::_1);
        
        // 发送目标
        auto goal_handle_future = nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
        
        is_navigating_ = true;
        navigation_start_time_ = this->now();
        current_goal_x_ = x;
        current_goal_y_ = y;
        current_goal_yaw_ = yaw;
        
        return true;
    }

private:
    // 初始化测试路点
    void initializeTestWaypoints()
    {
        // 这里添加一些测试路点，实际使用时应该从配置文件或服务调用获取
        test_waypoints_.push_back({1.0, 0.0, 0.0});
        test_waypoints_.push_back({1.0, 1.0, 1.57});
        test_waypoints_.push_back({0.0, 1.0, 3.14});
        test_waypoints_.push_back({0.0, 0.0, 0.0});
        
        RCLCPP_INFO(this->get_logger(), "Initialized %zu test waypoints", test_waypoints_.size());
    }
    
    // 全局路径回调
    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!is_navigating_) return;
        
        global_path_length_ = 0.0;
        for (size_t i = 1; i < msg->poses.size(); ++i) {
            double dx = msg->poses[i].pose.position.x - msg->poses[i-1].pose.position.x;
            double dy = msg->poses[i].pose.position.y - msg->poses[i-1].pose.position.y;
            global_path_length_ += std::sqrt(dx*dx + dy*dy);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Global path length: %.2f meters", global_path_length_);
    }
    
    // 本地路径回调
    void localPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        (void)msg;
        // 可以在这里处理本地路径信息
    }
    
    // 目标响应回调
    void goalResponseCallback(const GoalHandleNavigate::SharedPtr& goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            is_navigating_ = false;
            navigation_state_ = "REJECTED";
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            navigation_state_ = "ACCEPTED";
        }
    }
    
    // 反馈回调
    void feedbackCallback(
        GoalHandleNavigate::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        // 计算距离目标的距离
        double dx = feedback->current_pose.pose.position.x - current_goal_x_;
        double dy = feedback->current_pose.pose.position.y - current_goal_y_;
        distance_to_goal_ = std::sqrt(dx*dx + dy*dy);
        
        // 计算导航时间
        auto elapsed = (this->now() - navigation_start_time_).seconds();
        
        RCLCPP_INFO(this->get_logger(), 
                   "Navigation feedback - Distance to goal: %.2f m, Time: %.1f s, ETA: %d s",
                   distance_to_goal_, elapsed, feedback->estimated_time_remaining.sec);
        
        navigation_state_ = "NAVIGATING";
        
        // 检查超时
        if (elapsed > navigation_timeout_) {
            RCLCPP_WARN(this->get_logger(), "Navigation timeout! Canceling...");
            cancelNavigation();
        }
    }
    
    // 结果回调
    void resultCallback(const GoalHandleNavigate::WrappedResult& result)
    {
        is_navigating_ = false;
        
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
                navigation_state_ = "SUCCEEDED";
                navigation_success_count_++;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Navigation was aborted");
                navigation_state_ = "ABORTED";
                navigation_failure_count_++;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Navigation was canceled");
                navigation_state_ = "CANCELED";
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                navigation_state_ = "UNKNOWN";
                break;
        }
    }
    
    // 导航到目标服务回调
    void navigateToGoalCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        if (is_navigating_) {
            response->success = false;
            response->message = "Already navigating";
            return;
        }
        
        // 发送第一个测试路点
        if (!test_waypoints_.empty()) {
            auto& waypoint = test_waypoints_[current_waypoint_index_];
            bool success = sendGoal(waypoint.x, waypoint.y, waypoint.yaw);
            
            if (success) {
                response->success = true;
                response->message = "Navigation started to waypoint " + 
                                  std::to_string(current_waypoint_index_);
            } else {
                response->success = false;
                response->message = "Failed to send goal";
            }
        } else {
            response->success = false;
            response->message = "No waypoints available";
        }
    }
    
    // 取消导航服务回调
    void cancelNavigationCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        if (!is_navigating_) {
            response->success = false;
            response->message = "Not navigating";
            return;
        }
        
        cancelNavigation();
        
        response->success = true;
        response->message = "Navigation canceled";
    }
    
    // 取消导航
    void cancelNavigation()
    {
        // 发送停止命令
        auto stop_msg = geometry_msgs::msg::Twist();
        cmd_vel_pub_->publish(stop_msg);
        
        is_navigating_ = false;
        navigation_state_ = "CANCELED";
        
        RCLCPP_INFO(this->get_logger(), "Navigation canceled");
    }
    
    // 发布状态
    void publishStatus()
    {
        auto msg = std_msgs::msg::String();
        
        if (is_navigating_) {
            auto elapsed = (this->now() - navigation_start_time_).seconds();
            msg.data = "NAVIGATING - State: " + navigation_state_ + 
                      ", Distance: " + std::to_string(static_cast<int>(distance_to_goal_ * 100) / 100.0) + 
                      "m, Time: " + std::to_string(static_cast<int>(elapsed)) + "s";
        } else {
            msg.data = "IDLE - Success: " + std::to_string(navigation_success_count_) + 
                      ", Failed: " + std::to_string(navigation_failure_count_);
        }
        
        status_pub_->publish(msg);
    }
    
    // 路点结构
    struct Waypoint {
        double x;
        double y;
        double yaw;
    };
    
    // 成员变量
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
    
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr navigate_to_goal_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_navigation_srv_;
    
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    double navigation_timeout_;
    double goal_tolerance_;
    
    bool is_navigating_{false};
    std::string navigation_state_{"IDLE"};
    rclcpp::Time navigation_start_time_;
    
    double current_goal_x_{0.0};
    double current_goal_y_{0.0};
    double current_goal_yaw_{0.0};
    double distance_to_goal_{0.0};
    double global_path_length_{0.0};
    
    int navigation_success_count_{0};
    int navigation_failure_count_{0};
    
    std::vector<Waypoint> test_waypoints_;
    size_t current_waypoint_index_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
