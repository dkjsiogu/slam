#!/usr/bin/env python3
"""
里程计轨迹发布器
订阅 /odom 话题，将里程计数据转换为 Path 消息用于 RViz 可视化

功能:
- 订阅 /odom (nav_msgs/Odometry)
- 发布 /odom_path (nav_msgs/Path)
- 保留最近的轨迹点（可配置数量）
- 实时显示机器人移动轨迹

Author: Copilot
Date: 2025
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath(Node):
    """将里程计数据转换为轨迹路径"""
    
    def __init__(self):
        super().__init__('odom_to_path')
        
        # 参数
        self.declare_parameter('max_path_length', 1000)
        self.declare_parameter('path_topic', '/odom_path')
        self.declare_parameter('odom_topic', '/odom')
        
        self.max_path_length = self.get_parameter('max_path_length').value
        path_topic = self.get_parameter('path_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        
        # 轨迹存储
        self.path = Path()
        self.path.header.frame_id = 'odom'
        
        # 订阅和发布
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        
        # 统计
        self.point_count = 0
        
        self.get_logger().info(f'里程计轨迹发布器已启动')
        self.get_logger().info(f'  订阅: {odom_topic}')
        self.get_logger().info(f'  发布: {path_topic}')
        self.get_logger().info(f'  最大轨迹点数: {self.max_path_length}')
    
    def odom_callback(self, msg: Odometry):
        """接收里程计数据，添加到轨迹"""
        
        # 创建 PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        # 添加到轨迹
        self.path.poses.append(pose_stamped)
        self.point_count += 1
        
        # 限制轨迹长度
        if len(self.path.poses) > self.max_path_length:
            self.path.poses.pop(0)
        
        # 更新时间戳
        self.path.header.stamp = msg.header.stamp
        
        # 发布轨迹
        self.path_pub.publish(self.path)
        
        # 定期打印统计信息（每100个点）
        if self.point_count % 100 == 0:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.get_logger().info(
                f'已记录 {self.point_count} 个轨迹点，'
                f'当前位置: ({x:.3f}, {y:.3f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPath()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'总共记录了 {node.point_count} 个轨迹点')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
