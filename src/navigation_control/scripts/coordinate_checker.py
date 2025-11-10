#!/usr/bin/env python3
"""
坐标系验证脚本 - 检查机器人模型和TF是否对齐
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import math

class CoordinateSystemChecker(Node):
    def __init__(self):
        super().__init__('coordinate_checker')
        
        # 发布箭头标记到base_link前方
        self.marker_pub = self.create_publisher(Marker, '/coordinate_check', 10)
        
        # 定时发布
        self.timer = self.create_timer(1.0, self.publish_marker)
        
        self.get_logger().info('坐标系检查器已启动')
        self.get_logger().info('在RViz中添加 Marker 显示 /coordinate_check')
    
    def publish_marker(self):
        """发布一个箭头，指向base_link的+X方向（前方）"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "coordinate_check"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # 箭头从原点指向+X（前方）
        marker.points = []
        start = Marker().pose.position
        start.x = 0.0
        start.y = 0.0
        start.z = 0.1
        
        end = Marker().pose.position
        end.x = 0.3  # 30cm向前
        end.y = 0.0
        end.z = 0.1
        
        marker.points.append(start)
        marker.points.append(end)
        
        marker.scale.x = 0.02  # 箭头轴直径
        marker.scale.y = 0.04  # 箭头头部直径
        marker.scale.z = 0.06  # 箭头头部长度
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.marker_pub.publish(marker)
        
        self.get_logger().info('黄色箭头应该指向机器人前方（+X方向）', throttle_duration_sec=5.0)

def main():
    rclpy.init()
    node = CoordinateSystemChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
