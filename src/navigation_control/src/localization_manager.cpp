/**
 * @file localization_manager.cpp
 * @brief 定位模式管理器 - 管理AMCL定位、重定位、初始姿态设置
 * 
 * 功能:
 * 1. 启动/停止定位模式
 * 2. 加载已保存的地图
 * 3. 设置初始姿态 (重定位)
 * 4. 监控定位质量
 * 5. 发布定位状态
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;

class LocalizationManager : public rclcpp::Node
{
public:
    LocalizationManager() : Node("localization_manager"),
                           tf_buffer_(this->get_clock()),
                           tf_listener_(tf_buffer_)
    {
        // 声明参数
        this->declare_parameter("map_file", "");
        this->declare_parameter("initial_pose_x", 0.0);
        this->declare_parameter("initial_pose_y", 0.0);
        this->declare_parameter("initial_pose_yaw", 0.0);
        this->declare_parameter("localization_quality_threshold", 0.5);
        
        // 获取参数
        map_file_ = this->get_parameter("map_file").as_string();
        initial_pose_x_ = this->get_parameter("initial_pose_x").as_double();
        initial_pose_y_ = this->get_parameter("initial_pose_y").as_double();
        initial_pose_yaw_ = this->get_parameter("initial_pose_yaw").as_double();
        localization_threshold_ = this->get_parameter("localization_quality_threshold").as_double();
        
        // 发布器
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "localization_status", 10);
        
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);
        
        current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "current_pose", 10);
        
        // 订阅器 - AMCL粒子云 (用于评估定位质量)
        amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&LocalizationManager::amclPoseCallback, this, std::placeholders::_1));
        
        // 订阅激光扫描 (用于检测定位环境)
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LocalizationManager::scanCallback, this, std::placeholders::_1));
        
        // 服务
        start_localization_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "start_localization",
            std::bind(&LocalizationManager::startLocalizationCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        stop_localization_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "stop_localization",
            std::bind(&LocalizationManager::stopLocalizationCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        reset_localization_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "reset_localization",
            std::bind(&LocalizationManager::resetLocalizationCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // 定时器
        status_timer_ = this->create_wall_timer(
            1s, std::bind(&LocalizationManager::publishStatus, this));
        
        pose_timer_ = this->create_wall_timer(
            100ms, std::bind(&LocalizationManager::publishCurrentPose, this));
        
        quality_timer_ = this->create_wall_timer(
            2s, std::bind(&LocalizationManager::checkLocalizationQuality, this));
        
        RCLCPP_INFO(this->get_logger(), "Localization Manager initialized");
    }

private:
    // AMCL姿态回调
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!is_localizing_) return;
        
        last_amcl_pose_ = msg;
        last_pose_update_ = this->now();
        
        // 计算协方差 (定位质量评估)
        double cov_x = msg->pose.covariance[0];   // x方向方差
        double cov_y = msg->pose.covariance[7];   // y方向方差
        double cov_yaw = msg->pose.covariance[35]; // yaw方向方差
        
        localization_quality_ = 1.0 / (1.0 + std::sqrt(cov_x + cov_y + cov_yaw));
        
        // 检查是否定位良好
        if (localization_quality_ > localization_threshold_) {
            is_localized_ = true;
        }
    }
    
    // 激光扫描回调 (监控环境)
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        (void)msg;
        last_scan_time_ = this->now();
    }
    
    // 开始定位
    void startLocalizationCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        if (is_localizing_) {
            response->success = false;
            response->message = "Localization already started";
            return;
        }
        
        is_localizing_ = true;
        localization_start_time_ = this->now();
        
        // 发布初始姿态
        publishInitialPose();
        
        response->success = true;
        response->message = "Localization started successfully";
        
        RCLCPP_INFO(this->get_logger(), "Localization mode started");
    }
    
    // 停止定位
    void stopLocalizationCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        if (!is_localizing_) {
            response->success = false;
            response->message = "Localization not started";
            return;
        }
        
        is_localizing_ = false;
        is_localized_ = false;
        
        response->success = true;
        response->message = "Localization stopped successfully";
        
        RCLCPP_INFO(this->get_logger(), "Localization mode stopped");
    }
    
    // 重置定位 (重定位)
    void resetLocalizationCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        is_localized_ = false;
        publishInitialPose();
        
        response->success = true;
        response->message = "Localization reset successfully";
        
        RCLCPP_INFO(this->get_logger(), "Localization reset (relocalization)");
    }
    
    // 发布初始姿态
    void publishInitialPose()
    {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        
        msg.pose.pose.position.x = initial_pose_x_;
        msg.pose.pose.position.y = initial_pose_y_;
        msg.pose.pose.position.z = 0.0;
        
        // 将yaw转换为四元数
        tf2::Quaternion q;
        q.setRPY(0, 0, initial_pose_yaw_);
        msg.pose.pose.orientation = tf2::toMsg(q);
        
        // 设置协方差 (初始不确定性)
        msg.pose.covariance[0] = 0.25;  // x
        msg.pose.covariance[7] = 0.25;  // y
        msg.pose.covariance[35] = 0.068; // yaw (~15度)
        
        initial_pose_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), 
                   "Published initial pose: x=%.2f, y=%.2f, yaw=%.2f",
                   initial_pose_x_, initial_pose_y_, initial_pose_yaw_);
    }
    
    // 发布当前姿态 (从TF获取)
    void publishCurrentPose()
    {
        if (!is_localizing_) return;
        
        try {
            // 从TF树获取map->base_link变换
            auto transform = tf_buffer_.lookupTransform(
                "map", "base_link", tf2::TimePointZero);
            
            auto msg = geometry_msgs::msg::PoseStamped();
            msg.header = transform.header;
            msg.pose.position.x = transform.transform.translation.x;
            msg.pose.position.y = transform.transform.translation.y;
            msg.pose.position.z = transform.transform.translation.z;
            msg.pose.orientation = transform.transform.rotation;
            
            current_pose_pub_->publish(msg);
            
        } catch (tf2::TransformException& ex) {
            RCLCPP_DEBUG(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }
    
    // 检查定位质量
    void checkLocalizationQuality()
    {
        if (!is_localizing_) return;
        
        auto now = this->now();
        
        // 检查是否长时间没有收到AMCL姿态
        if ((now - last_pose_update_).seconds() > 5.0) {
            RCLCPP_WARN(this->get_logger(), 
                       "No AMCL pose received for 5 seconds - localization may have failed");
            is_localized_ = false;
        }
        
        // 检查定位质量
        if (is_localized_) {
            RCLCPP_INFO(this->get_logger(), 
                       "Localization quality: %.2f (threshold: %.2f)",
                       localization_quality_, localization_threshold_);
        }
    }
    
    // 发布状态
    void publishStatus()
    {
        auto msg = std_msgs::msg::String();
        
        if (is_localizing_) {
            std::string status = is_localized_ ? "LOCALIZED" : "LOCALIZING";
            msg.data = status + " - Quality: " + 
                      std::to_string(static_cast<int>(localization_quality_ * 100)) + "%";
        } else {
            msg.data = "IDLE";
        }
        
        status_pub_->publish(msg);
    }
    
    // 成员变量
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_localization_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_localization_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_localization_srv_;
    
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr pose_timer_;
    rclcpp::TimerBase::SharedPtr quality_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    std::string map_file_;
    double initial_pose_x_;
    double initial_pose_y_;
    double initial_pose_yaw_;
    double localization_threshold_;
    
    bool is_localizing_{false};
    bool is_localized_{false};
    double localization_quality_{0.0};
    
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_amcl_pose_;
    rclcpp::Time localization_start_time_;
    rclcpp::Time last_pose_update_;
    rclcpp::Time last_scan_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
