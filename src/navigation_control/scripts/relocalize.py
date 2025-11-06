#!/usr/bin/env python3
"""
全局重定位工具
用于在机器人位姿丢失时触发 AMCL 全局重定位
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import math
import sys

class GlobalRelocalizer(Node):
    def __init__(self, x=None, y=None, yaw=None):
        super().__init__('global_relocalizer')
        
        self.target_x = x
        self.target_y = y
        self.target_yaw = yaw if yaw is not None else 0.0
        
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # 订阅地图获取信息
        qos_transient = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_transient
        )
        
        self.map_info = None
        self.get_logger().info('等待地图信息...')
    
    def map_callback(self, msg):
        if self.map_info is None:
            self.map_info = msg.info
            self.get_logger().info(f'收到地图: {msg.info.width}x{msg.info.height}')
            self.get_logger().info(f'原点: [{msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f}]')
            
            # 如果没有指定位置，使用地图中心
            if self.target_x is None or self.target_y is None:
                center_x = msg.info.origin.position.x + (msg.info.width * msg.info.resolution) / 2.0
                center_y = msg.info.origin.position.y + (msg.info.height * msg.info.resolution) / 2.0
                self.target_x = center_x
                self.target_y = center_y
                self.get_logger().info(f'使用地图中心: [{center_x:.2f}, {center_y:.2f}]')
            
            # 发布重定位位姿
            self.create_timer(0.5, self.publish_pose)
    
    def publish_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.pose.position.x = self.target_x
        pose_msg.pose.pose.position.y = self.target_y
        pose_msg.pose.pose.position.z = 0.0
        
        # 四元数
        yaw = self.target_yaw
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # 大协方差用于全局重定位
        pose_msg.pose.covariance = [0.0] * 36
        pose_msg.pose.covariance[0] = 0.8   # x
        pose_msg.pose.covariance[7] = 0.8   # y
        pose_msg.pose.covariance[35] = 0.5  # yaw
        
        for _ in range(5):
            self.pose_pub.publish(pose_msg)
        
        self.get_logger().info(f'✓ 已发布重定位位姿: x={self.target_x:.2f}, y={self.target_y:.2f}, yaw={self.target_yaw:.2f}')
        self.get_logger().info('AMCL 将使用全局重定位恢复位姿估计')
        
        # 完成后关闭
        self.create_timer(1.0, lambda: self.destroy_node() or rclpy.shutdown())

def main(args=None):
    rclpy.init(args=args)
    
    # 解析命令行参数
    x = None
    y = None
    yaw = None
    
    if len(sys.argv) > 1:
        try:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            if len(sys.argv) > 3:
                yaw = float(sys.argv[3])
            else:
                yaw = 0.0
        except (ValueError, IndexError):
            print("用法: relocalize.py [x y [yaw]]")
            print("示例: relocalize.py 1.9 0.1 0.0")
            print("不提供参数则使用地图中心")
            return
    
    node = GlobalRelocalizer(x, y, yaw)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
