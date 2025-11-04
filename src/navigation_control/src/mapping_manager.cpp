/**
 * @file mapping_manager.cpp
 * @brief 建图模式管理器 - 管理Cartographer建图、地图保存等功能
 * 
 * 功能:
 * 1. 启动/停止建图模式
 * 2. 保存地图 (调用map_saver服务)
 * 3. 完成建图后的处理
 * 4. 发布建图状态
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/srv/save_map.hpp>
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class MappingManager : public rclcpp::Node
{
public:
    MappingManager() : Node("mapping_manager")
    {
        // 声明参数
        this->declare_parameter("map_save_directory", "/home/dkjsiogu/slam/src/navigation_control/maps");
        this->declare_parameter("map_name", "my_map");
        this->declare_parameter("auto_save_interval", 60.0);  // 自动保存间隔(秒)
        
        // 获取参数
        map_save_directory_ = this->get_parameter("map_save_directory").as_string();
        map_name_ = this->get_parameter("map_name").as_string();
        auto_save_interval_ = this->get_parameter("auto_save_interval").as_double();
        
        // 状态发布器
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "mapping_status", 10);
        
        // 地图订阅器 (监控地图更新)
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&MappingManager::mapCallback, this, std::placeholders::_1));
        
        // 服务客户端 - 保存地图
        map_saver_client_ = this->create_client<nav2_msgs::srv::SaveMap>(
            "/map_saver/save_map");
        
        // 服务服务器 - 外部控制
        start_mapping_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "start_mapping",
            std::bind(&MappingManager::startMappingCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        stop_mapping_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "stop_mapping",
            std::bind(&MappingManager::stopMappingCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        save_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "save_map",
            std::bind(&MappingManager::saveMapCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // 定时器 - 自动保存地图
        if (auto_save_interval_ > 0) {
            auto_save_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(auto_save_interval_),
                std::bind(&MappingManager::autoSaveTimerCallback, this));
        }
        
        // 状态发布定时器
        status_timer_ = this->create_wall_timer(
            1s, std::bind(&MappingManager::publishStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Mapping Manager initialized");
        RCLCPP_INFO(this->get_logger(), "Map save directory: %s", map_save_directory_.c_str());
        RCLCPP_INFO(this->get_logger(), "Auto-save interval: %.1f seconds", auto_save_interval_);
    }

private:
    // 地图回调 - 更新地图统计信息
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (!is_mapping_) return;
        
        last_map_update_ = this->now();
        map_updates_count_++;
        
        // 统计已知区域
        int known_cells = 0;
        for (const auto& cell : msg->data) {
            if (cell >= 0) known_cells++;
        }
        
        map_coverage_ = static_cast<double>(known_cells) / msg->data.size() * 100.0;
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Map updated - Coverage: %.2f%%, Updates: %d",
                    map_coverage_, map_updates_count_);
    }
    
    // 开始建图
    void startMappingCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        if (is_mapping_) {
            response->success = false;
            response->message = "Mapping already started";
            return;
        }
        
        is_mapping_ = true;
        mapping_start_time_ = this->now();
        map_updates_count_ = 0;
        
        response->success = true;
        response->message = "Mapping started successfully";
        
        RCLCPP_INFO(this->get_logger(), "Mapping mode started");
    }
    
    // 停止建图
    void stopMappingCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        if (!is_mapping_) {
            response->success = false;
            response->message = "Mapping not started";
            return;
        }
        
        is_mapping_ = false;
        
        response->success = true;
        response->message = "Mapping stopped successfully";
        
        RCLCPP_INFO(this->get_logger(), "Mapping mode stopped");
        RCLCPP_INFO(this->get_logger(), "Total map updates: %d", map_updates_count_);
    }
    
    // 保存地图
    void saveMapCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        if (!saveMap()) {
            response->success = false;
            response->message = "Failed to save map";
        } else {
            response->success = true;
            response->message = "Map saved successfully";
        }
    }
    
    // 实际保存地图的函数
    bool saveMap()
    {
        if (!map_saver_client_->wait_for_service(2s)) {
            RCLCPP_ERROR(this->get_logger(), "Map saver service not available");
            return false;
        }
        
        auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
        
        // 生成带时间戳的地图名称
        auto now = this->now();
        std::string timestamp = std::to_string(now.seconds());
        
        request->map_topic = "/map";
        request->map_url = map_save_directory_ + "/" + map_name_ + "_" + timestamp;
        request->image_format = "pgm";
        request->map_mode = "trinary";  // 三值模式: 占用/空闲/未知
        request->free_thresh = 0.25;
        request->occupied_thresh = 0.65;
        
        RCLCPP_INFO(this->get_logger(), "Saving map to: %s", request->map_url.c_str());
        
        auto result = map_saver_client_->async_send_request(request);
        
        // 等待结果
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Map saved successfully!");
            last_saved_map_ = request->map_url;
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save map");
            return false;
        }
    }
    
    // 自动保存定时器
    void autoSaveTimerCallback()
    {
        if (is_mapping_ && map_updates_count_ > 0) {
            RCLCPP_INFO(this->get_logger(), "Auto-saving map...");
            saveMap();
        }
    }
    
    // 发布状态
    void publishStatus()
    {
        auto msg = std_msgs::msg::String();
        
        if (is_mapping_) {
            auto elapsed = (this->now() - mapping_start_time_).seconds();
            msg.data = "MAPPING - Elapsed: " + std::to_string(static_cast<int>(elapsed)) + 
                      "s, Coverage: " + std::to_string(static_cast<int>(map_coverage_)) + 
                      "%, Updates: " + std::to_string(map_updates_count_);
        } else {
            msg.data = "IDLE";
        }
        
        status_pub_->publish(msg);
    }
    
    // 成员变量
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr map_saver_client_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mapping_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_mapping_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
    
    rclcpp::TimerBase::SharedPtr auto_save_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    std::string map_save_directory_;
    std::string map_name_;
    std::string last_saved_map_;
    double auto_save_interval_;
    
    bool is_mapping_{false};
    int map_updates_count_{0};
    double map_coverage_{0.0};
    rclcpp::Time mapping_start_time_;
    rclcpp::Time last_map_update_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MappingManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
