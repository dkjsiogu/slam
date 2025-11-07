#!/usr/bin/env python3
"""
里程计数据可视化查看器
显示累积的位姿和实时速度
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import math

class OdomDataViewer(Node):
    def __init__(self):
        super().__init__('odom_data_viewer')
        
        # 订阅里程计话题
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 订阅原始增量数据
        self.raw_odom_sub = self.create_subscription(
            Float32MultiArray,
            '/wheel_odom_data',
            self.raw_odom_callback,
            10
        )
        
        # 定时器 - 显示数据 (1Hz)
        self.timer = self.create_timer(1.0, self.display_data)
        
        # 数据存储
        self.latest_odom = None
        self.latest_raw = None
        
        self.get_logger().info('里程计数据查看器已启动')
        self.get_logger().info('=' * 100)
    
    def odom_callback(self, msg):
        """接收里程计消息"""
        self.latest_odom = msg
    
    def raw_odom_callback(self, msg):
        """接收原始增量数据"""
        self.latest_raw = msg.data
    
    def display_data(self):
        """显示数据"""
        if self.latest_odom is None:
            self.get_logger().warn('等待里程计数据...')
            return
        
        # 提取位姿
        x = self.latest_odom.pose.pose.position.x
        y = self.latest_odom.pose.pose.position.y
        
        # 提取姿态 (四元数转欧拉角)
        quat = self.latest_odom.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        # 提取速度
        vx = self.latest_odom.twist.twist.linear.x
        vy = self.latest_odom.twist.twist.linear.y
        wz = self.latest_odom.twist.twist.angular.z
        
        # 显示数据
        self.get_logger().info('')
        self.get_logger().info('╔' + '═' * 98 + '╗')
        self.get_logger().info(f'║ {"里程计状态":^96s} ║')
        self.get_logger().info('╠' + '═' * 98 + '╣')
        self.get_logger().info(f'║ 累积位姿:  X = {x:8.3f} m    Y = {y:8.3f} m    θ = {math.degrees(theta):7.2f}°{" " * 38}║')
        self.get_logger().info(f'║ 瞬时速度:  Vx= {vx:7.3f} m/s  Vy= {vy:7.3f} m/s  Wz= {wz:7.3f} rad/s{" " * 31}║')
        
        # 显示原始增量数据
        if self.latest_raw is not None and len(self.latest_raw) >= 4:
            dx = self.latest_raw[0]
            dy = self.latest_raw[1]
            dw = self.latest_raw[2]
            dt = self.latest_raw[3]
            self.get_logger().info('╠' + '─' * 98 + '╣')
            self.get_logger().info(f'║ 最新增量:  dx= {dx:7.4f} m   dy= {dy:7.4f} m   dθ= {dw:7.4f} rad  dt= {dt:6.3f} s{" " * 26}║')
        
        self.get_logger().info('╚' + '═' * 98 + '╝')
        self.get_logger().info('')

def main(args=None):
    rclpy.init(args=args)
    node = OdomDataViewer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
