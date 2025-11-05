/**
 * @file manual_goal_setter.cpp
 * @brief 手动目标点设置节点 - 用于路径规划调试
 * 
 * 功能:
 * 1. 提供服务接口手动设置导航目标点
 * 2. 预设多个测试点位
 * 3. 支持从命令行或RViz设置目标
 * 4. 目标点序列执行（巡航模式）
 * 5. 目标点可视化
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <string>
#include <memory>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// 目标点结构
struct Waypoint {
    std::string name;
    double x;
    double y;
    double yaw;  // 弧度
};

class ManualGoalSetter : public rclcpp::Node
{
public:
    ManualGoalSetter() : Node("manual_goal_setter")
    {
        // 声明参数
        this->declare_parameter("goal_tolerance", 0.3);
        this->declare_parameter("enable_visualization", true);
        
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        enable_visualization_ = this->get_parameter("enable_visualization").as_bool();
        
        // Action客户端
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");
        
        // 订阅RViz的目标点 (2D Nav Goal)
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&ManualGoalSetter::goalPoseCallback, this, std::placeholders::_1));
        
        // 发布状态信息
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "goal_setter_status", 10);
        
        // 发布目标点可视化
        if (enable_visualization_) {
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "waypoint_markers", 10);
        }
        
        // 服务 - 设置目标点
        set_goal_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "set_goal_0",
            std::bind(&ManualGoalSetter::setGoal0Callback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        set_goal_1_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "set_goal_1",
            std::bind(&ManualGoalSetter::setGoal1Callback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        set_goal_2_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "set_goal_2",
            std::bind(&ManualGoalSetter::setGoal2Callback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        set_goal_3_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "set_goal_3",
            std::bind(&ManualGoalSetter::setGoal3Callback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // 服务 - 开始巡航
        start_patrol_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "start_patrol",
            std::bind(&ManualGoalSetter::startPatrolCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // 服务 - 停止导航
        cancel_goal_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "cancel_goal",
            std::bind(&ManualGoalSetter::cancelGoalCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // 定时器 - 发布可视化
        if (enable_visualization_) {
            viz_timer_ = this->create_wall_timer(
                1s, std::bind(&ManualGoalSetter::publishVisualization, this));
        }
        
        // 初始化预设目标点
        initializeWaypoints();
        
        RCLCPP_INFO(this->get_logger(), "手动目标点设置节点已初始化");
        RCLCPP_INFO(this->get_logger(), "已加载 %zu 个预设目标点", waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "=================================");
        RCLCPP_INFO(this->get_logger(), "可用服务:");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /set_goal_0 std_srvs/srv/Trigger");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /set_goal_1 std_srvs/srv/Trigger");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /set_goal_2 std_srvs/srv/Trigger");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /set_goal_3 std_srvs/srv/Trigger");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /start_patrol std_srvs/srv/Trigger");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /cancel_goal std_srvs/srv/Trigger");
    }

private:
    // 初始化预设目标点（根据实际环境修改）
    void initializeWaypoints()
    {
        // 这些是示例点位，需要根据实际地图调整
        waypoints_.push_back({"原点", 0.0, 0.0, 0.0});
        waypoints_.push_back({"前方1米", 1.0, 0.0, 0.0});
        waypoints_.push_back({"左前方", 1.0, 1.0, M_PI/4});
        waypoints_.push_back({"左侧", 0.0, 1.0, M_PI/2});
        
        RCLCPP_INFO(this->get_logger(), "预设目标点:");
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  [%zu] %s: (%.2f, %.2f, %.2f rad)",
                       i, waypoints_[i].name.c_str(),
                       waypoints_[i].x, waypoints_[i].y, waypoints_[i].yaw);
        }
    }
    
    // 发送目标点
    bool sendGoal(const Waypoint& waypoint)
    {
        if (!nav_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "导航服务器不可用");
            return false;
        }
        
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";
        
        goal_msg.pose.pose.position.x = waypoint.x;
        goal_msg.pose.pose.position.y = waypoint.y;
        goal_msg.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, waypoint.yaw);
        goal_msg.pose.pose.orientation = tf2::toMsg(q);
        
        RCLCPP_INFO(this->get_logger(), 
                   "发送目标点 '%s': x=%.2f, y=%.2f, yaw=%.2f",
                   waypoint.name.c_str(), waypoint.x, waypoint.y, waypoint.yaw);
        
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ManualGoalSetter::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&ManualGoalSetter::resultCallback, this, std::placeholders::_1);
        
        current_goal_ = waypoint.name;
        nav_client_->async_send_goal(goal_msg, send_goal_options);
        
        return true;
    }
    
    // RViz目标点回调
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 从四元数提取yaw
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        Waypoint wp;
        wp.name = "RViz目标";
        wp.x = msg->pose.position.x;
        wp.y = msg->pose.position.y;
        wp.yaw = yaw;
        
        RCLCPP_INFO(this->get_logger(), "收到RViz目标点");
        sendGoal(wp);
    }
    
    // 目标响应回调
    void goalResponseCallback(const GoalHandleNavigate::SharedPtr& goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "目标被服务器拒绝");
            current_goal_ = "";
        } else {
            RCLCPP_INFO(this->get_logger(), "目标已被服务器接受");
        }
    }
    
    // 结果回调
    void resultCallback(const GoalHandleNavigate::WrappedResult& result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "✓ 成功到达目标点: %s", current_goal_.c_str());
                
                // 如果在巡航模式，继续下一个点
                if (is_patrolling_) {
                    current_waypoint_index_++;
                    if (current_waypoint_index_ >= waypoints_.size()) {
                        current_waypoint_index_ = 0;  // 循环
                    }
                    
                    // 等待2秒后前往下一个点
                    patrol_timer_ = this->create_wall_timer(
                        2s, [this]() {
                            sendGoal(waypoints_[current_waypoint_index_]);
                            patrol_timer_->cancel();
                        });
                }
                break;
                
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "✗ 导航失败: %s", current_goal_.c_str());
                is_patrolling_ = false;
                break;
                
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "⊗ 导航已取消: %s", current_goal_.c_str());
                is_patrolling_ = false;
                break;
                
            default:
                RCLCPP_ERROR(this->get_logger(), "未知结果状态");
                is_patrolling_ = false;
                break;
        }
        
        current_goal_ = "";
    }
    
    // 服务回调 - 设置目标点0
    void setGoal0Callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (waypoints_.size() > 0) {
            response->success = sendGoal(waypoints_[0]);
            response->message = "发送目标点: " + waypoints_[0].name;
        } else {
            response->success = false;
            response->message = "没有可用的目标点";
        }
    }
    
    void setGoal1Callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (waypoints_.size() > 1) {
            response->success = sendGoal(waypoints_[1]);
            response->message = "发送目标点: " + waypoints_[1].name;
        } else {
            response->success = false;
            response->message = "目标点1不存在";
        }
    }
    
    void setGoal2Callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (waypoints_.size() > 2) {
            response->success = sendGoal(waypoints_[2]);
            response->message = "发送目标点: " + waypoints_[2].name;
        } else {
            response->success = false;
            response->message = "目标点2不存在";
        }
    }
    
    void setGoal3Callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (waypoints_.size() > 3) {
            response->success = sendGoal(waypoints_[3]);
            response->message = "发送目标点: " + waypoints_[3].name;
        } else {
            response->success = false;
            response->message = "目标点3不存在";
        }
    }
    
    // 开始巡航
    void startPatrolCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (waypoints_.empty()) {
            response->success = false;
            response->message = "没有可用的巡航点";
            return;
        }
        
        is_patrolling_ = true;
        current_waypoint_index_ = 0;
        
        response->success = sendGoal(waypoints_[current_waypoint_index_]);
        response->message = "开始巡航模式，共 " + std::to_string(waypoints_.size()) + " 个点";
        
        RCLCPP_INFO(this->get_logger(), "开始巡航，共 %zu 个点", waypoints_.size());
    }
    
    // 取消导航
    void cancelGoalCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        is_patrolling_ = false;
        
        // TODO: 取消当前导航任务
        
        response->success = true;
        response->message = "已停止导航和巡航";
        
        RCLCPP_INFO(this->get_logger(), "已停止导航");
    }
    
    // 发布可视化标记
    void publishVisualization()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // 显示所有预设点
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            // 箭头标记
            visualization_msgs::msg::Marker arrow;
            arrow.header.frame_id = "map";
            arrow.header.stamp = this->now();
            arrow.ns = "waypoints";
            arrow.id = i * 2;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            
            arrow.pose.position.x = waypoints_[i].x;
            arrow.pose.position.y = waypoints_[i].y;
            arrow.pose.position.z = 0.0;
            
            tf2::Quaternion q;
            q.setRPY(0, 0, waypoints_[i].yaw);
            arrow.pose.orientation = tf2::toMsg(q);
            
            arrow.scale.x = 0.5;
            arrow.scale.y = 0.1;
            arrow.scale.z = 0.1;
            
            arrow.color.r = 0.0;
            arrow.color.g = 1.0;
            arrow.color.b = 0.0;
            arrow.color.a = 0.8;
            
            marker_array.markers.push_back(arrow);
            
            // 文本标记
            visualization_msgs::msg::Marker text;
            text.header.frame_id = "map";
            text.header.stamp = this->now();
            text.ns = "waypoint_labels";
            text.id = i * 2 + 1;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            
            text.pose.position.x = waypoints_[i].x;
            text.pose.position.y = waypoints_[i].y;
            text.pose.position.z = 0.5;
            text.pose.orientation.w = 1.0;
            
            text.scale.z = 0.2;
            
            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;
            text.color.a = 1.0;
            
            text.text = "[" + std::to_string(i) + "] " + waypoints_[i].name;
            
            marker_array.markers.push_back(text);
        }
        
        marker_pub_->publish(marker_array);
    }
    
    // 成员变量
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_goal_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_goal_1_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_goal_2_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_goal_3_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_patrol_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_goal_srv_;
    
    rclcpp::TimerBase::SharedPtr viz_timer_;
    rclcpp::TimerBase::SharedPtr patrol_timer_;
    
    std::vector<Waypoint> waypoints_;
    std::string current_goal_;
    bool is_patrolling_{false};
    size_t current_waypoint_index_{0};
    
    double goal_tolerance_;
    bool enable_visualization_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManualGoalSetter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
