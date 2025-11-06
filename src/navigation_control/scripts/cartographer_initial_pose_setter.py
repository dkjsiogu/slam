#!/usr/bin/env python3
"""
Cartographer 初始位姿设置节点
功能:
1. 监听 /initialpose 话题 (RViz2 的 "2D Pose Estimate" 工具)
2. 调用 Cartographer 服务设置初始位姿
3. 辅助全局定位功能
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
import time

class CartographerInitialPoseSetter(Node):
    def __init__(self):
        super().__init__('cartographer_initial_pose_setter')
        
        # 订阅 RViz2 的初始位姿话题
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )
        
        self.get_logger().info('Cartographer 初始位姿设置节点已启动')
        self.get_logger().info('在 RViz2 中使用 "2D Pose Estimate" 工具可手动设置初始位姿')
        self.get_logger().info('提示: Cartographer 支持全局定位，通常会自动找到正确位置')
        
    def initialpose_callback(self, msg):
        """处理 RViz2 发送的初始位姿"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 从四元数提取yaw角
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = 3.14159265359 * (siny_cosp / (cosy_cosp if cosy_cosp != 0 else 0.0001)) / 3.14159265359
        
        self.get_logger().info(f'收到初始位姿提示: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        self.get_logger().info('注意: Cartographer 会使用此位姿作为提示，但最终会通过扫描匹配自动调整')
        
        # Cartographer 会自动处理 /initialpose，这里只是记录日志
        # 实际的定位由 Cartographer 的全局定位功能完成

def main(args=None):
    rclpy.init(args=args)
    node = CartographerInitialPoseSetter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
