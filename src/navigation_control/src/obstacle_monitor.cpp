/**
 * @file obstacle_monitor.cpp
 * @brief 障碍物监控与距离显示节点
 * 
 * 功能:
 * 1. 订阅激光雷达数据 (/scan)
 * 2. 计算最近障碍物距离（米为单位）
 * 3. 分区域显示障碍物距离（前、后、左、右）
 * 4. 发布可视化标记到RViz
 * 5. 安全区域检测和警告
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

using namespace std::chrono_literals;

class ObstacleMonitor : public rclcpp::Node
{
public:
    ObstacleMonitor() : Node("obstacle_monitor")
    {
        // 声明参数
        this->declare_parameter("warning_distance", 0.5);      // 警告距离 (米)
        this->declare_parameter("danger_distance", 0.3);       // 危险距离 (米)
        this->declare_parameter("scan_topic", "/scan");        // 激光雷达话题
        this->declare_parameter("update_rate", 10.0);          // 更新频率 (Hz)
        this->declare_parameter("enable_visualization", true); // 启用可视化
        
        warning_distance_ = this->get_parameter("warning_distance").as_double();
        danger_distance_ = this->get_parameter("danger_distance").as_double();
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        update_rate_ = this->get_parameter("update_rate").as_double();
        enable_visualization_ = this->get_parameter("enable_visualization").as_bool();
        
        // 订阅激光雷达数据
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ObstacleMonitor::scanCallback, this, std::placeholders::_1));
        
        // 发布障碍物信息
        obstacle_info_pub_ = this->create_publisher<std_msgs::msg::String>(
            "obstacle_info", 10);
        
        // 发布最近障碍物距离
        min_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "min_obstacle_distance", 10);
        
        // 发布各方向障碍物距离
        front_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "front_obstacle_distance", 10);
        back_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "back_obstacle_distance", 10);
        left_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "left_obstacle_distance", 10);
        right_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "right_obstacle_distance", 10);
        
        // 发布可视化标记
        if (enable_visualization_) {
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "obstacle_markers", 10);
        }
        
        // 定时器 - 发布状态信息
        status_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / update_rate_),
            std::bind(&ObstacleMonitor::publishStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "障碍物监控节点已初始化");
        RCLCPP_INFO(this->get_logger(), "警告距离: %.2f 米", warning_distance_);
        RCLCPP_INFO(this->get_logger(), "危险距离: %.2f 米", danger_distance_);
    }

private:
    // 激光雷达数据回调
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_ = msg;
        
        // 重置距离数据
        min_distance_ = std::numeric_limits<double>::max();
        front_distance_ = std::numeric_limits<double>::max();
        back_distance_ = std::numeric_limits<double>::max();
        left_distance_ = std::numeric_limits<double>::max();
        right_distance_ = std::numeric_limits<double>::max();
        
        size_t num_readings = msg->ranges.size();
        if (num_readings == 0) return;
        
        // 计算各方向的角度范围
        // 前方: -45° ~ +45°
        // 右侧: -135° ~ -45°
        // 后方: ±135° ~ ±180°
        // 左侧: +45° ~ +135°
        
        for (size_t i = 0; i < num_readings; ++i) {
            float range = msg->ranges[i];
            
            // 过滤无效数据
            if (std::isnan(range) || std::isinf(range) || 
                range < msg->range_min || range > msg->range_max) {
                continue;
            }
            
            // 计算当前点的角度
            double angle = msg->angle_min + i * msg->angle_increment;
            
            // 更新最小距离
            if (range < min_distance_) {
                min_distance_ = range;
                min_distance_angle_ = angle;
            }
            
            // 根据角度分类到不同方向
            double angle_deg = angle * 180.0 / M_PI;
            
            if (angle_deg >= -45.0 && angle_deg <= 45.0) {
                // 前方
                if (range < front_distance_) {
                    front_distance_ = range;
                }
            } else if (angle_deg > 45.0 && angle_deg <= 135.0) {
                // 左侧
                if (range < left_distance_) {
                    left_distance_ = range;
                }
            } else if (angle_deg < -45.0 && angle_deg >= -135.0) {
                // 右侧
                if (range < right_distance_) {
                    right_distance_ = range;
                }
            } else {
                // 后方
                if (range < back_distance_) {
                    back_distance_ = range;
                }
            }
        }
        
        // 检查安全状态
        checkSafety();
        
        // 发布可视化
        if (enable_visualization_) {
            publishVisualization();
        }
    }
    
    // 检查安全状态
    void checkSafety()
    {
        if (min_distance_ < danger_distance_) {
            safety_status_ = "DANGER";
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "危险！最近障碍物距离: %.3f 米", min_distance_);
        } else if (min_distance_ < warning_distance_) {
            safety_status_ = "WARNING";
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "警告！最近障碍物距离: %.3f 米", min_distance_);
        } else {
            safety_status_ = "SAFE";
        }
    }
    
    // 发布状态信息
    void publishStatus()
    {
        if (!last_scan_) return;
        
        // 发布各方向距离
        auto min_msg = std_msgs::msg::Float32();
        min_msg.data = (min_distance_ == std::numeric_limits<double>::max()) ? -1.0 : min_distance_;
        min_distance_pub_->publish(min_msg);
        
        auto front_msg = std_msgs::msg::Float32();
        front_msg.data = (front_distance_ == std::numeric_limits<double>::max()) ? -1.0 : front_distance_;
        front_distance_pub_->publish(front_msg);
        
        auto back_msg = std_msgs::msg::Float32();
        back_msg.data = (back_distance_ == std::numeric_limits<double>::max()) ? -1.0 : back_distance_;
        back_distance_pub_->publish(back_msg);
        
        auto left_msg = std_msgs::msg::Float32();
        left_msg.data = (left_distance_ == std::numeric_limits<double>::max()) ? -1.0 : left_distance_;
        left_distance_pub_->publish(left_msg);
        
        auto right_msg = std_msgs::msg::Float32();
        right_msg.data = (right_distance_ == std::numeric_limits<double>::max()) ? -1.0 : right_distance_;
        right_distance_pub_->publish(right_msg);
        
        // 发布综合信息
        auto info_msg = std_msgs::msg::String();
        char buffer[512];
        snprintf(buffer, sizeof(buffer),
                "障碍物距离 [米] | 状态: %s | "
                "最近: %.3f | 前: %.3f | 后: %.3f | 左: %.3f | 右: %.3f",
                safety_status_.c_str(),
                min_distance_ == std::numeric_limits<double>::max() ? -1.0 : min_distance_,
                front_distance_ == std::numeric_limits<double>::max() ? -1.0 : front_distance_,
                back_distance_ == std::numeric_limits<double>::max() ? -1.0 : back_distance_,
                left_distance_ == std::numeric_limits<double>::max() ? -1.0 : left_distance_,
                right_distance_ == std::numeric_limits<double>::max() ? -1.0 : right_distance_);
        info_msg.data = buffer;
        obstacle_info_pub_->publish(info_msg);
    }
    
    // 发布可视化标记
    void publishVisualization()
    {
        if (!last_scan_) return;
        
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // 删除旧标记
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = last_scan_->header.frame_id;
        delete_marker.header.stamp = this->now();
        delete_marker.ns = "obstacles";
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);
        
        // 创建文本标记显示各方向距离
        std::vector<std::string> directions = {"前", "左", "后", "右"};
        std::vector<double> distances = {front_distance_, left_distance_, back_distance_, right_distance_};
        std::vector<std::pair<double, double>> positions = {{0.5, 0.0}, {0.0, 0.5}, {-0.5, 0.0}, {0.0, -0.5}};
        
        for (size_t i = 0; i < directions.size(); ++i) {
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = "base_link";
            text_marker.header.stamp = this->now();
            text_marker.ns = "obstacle_distances";
            text_marker.id = i;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            text_marker.pose.position.x = positions[i].first;
            text_marker.pose.position.y = positions[i].second;
            text_marker.pose.position.z = 0.3;
            text_marker.pose.orientation.w = 1.0;
            
            text_marker.scale.z = 0.1;
            
            // 根据距离设置颜色
            double dist = distances[i];
            if (dist < danger_distance_) {
                text_marker.color.r = 1.0;
                text_marker.color.g = 0.0;
                text_marker.color.b = 0.0;
            } else if (dist < warning_distance_) {
                text_marker.color.r = 1.0;
                text_marker.color.g = 0.5;
                text_marker.color.b = 0.0;
            } else {
                text_marker.color.r = 0.0;
                text_marker.color.g = 1.0;
                text_marker.color.b = 0.0;
            }
            text_marker.color.a = 1.0;
            
            if (dist == std::numeric_limits<double>::max()) {
                text_marker.text = directions[i] + ": --";
            } else {
                char text_buf[64];
                snprintf(text_buf, sizeof(text_buf), "%s: %.2fm", directions[i].c_str(), dist);
                text_marker.text = text_buf;
            }
            
            text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            marker_array.markers.push_back(text_marker);
        }
        
        marker_pub_->publish(marker_array);
    }
    
    // 成员变量
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_info_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr min_distance_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_distance_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr back_distance_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_distance_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_distance_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    
    double warning_distance_;
    double danger_distance_;
    std::string scan_topic_;
    double update_rate_;
    bool enable_visualization_;
    
    double min_distance_{std::numeric_limits<double>::max()};
    double min_distance_angle_{0.0};
    double front_distance_{std::numeric_limits<double>::max()};
    double back_distance_{std::numeric_limits<double>::max()};
    double left_distance_{std::numeric_limits<double>::max()};
    double right_distance_{std::numeric_limits<double>::max()};
    
    std::string safety_status_{"UNKNOWN"};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
