#!/usr/bin/env python3
"""
测试速度发布器
用于调试时发布测试速度命令

键盘控制:
- w/s: 前进/后退
- a/d: 左/右平移  
- q/e: 左/右旋转
- space: 停止
- Ctrl+C: 退出
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty


class TestVelocityPublisher(Node):
    def __init__(self):
        super().__init__('test_velocity_publisher')
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.declare_parameter('linear_step', 0.1)
        self.declare_parameter('angular_step', 0.2)
        
        self.linear_step = self.get_parameter('linear_step').value
        self.angular_step = self.get_parameter('angular_step').value
        
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        
        self.get_logger().info('测试速度发布器已启动')
        self.get_logger().info('按键控制: w/s=前后, a/d=左右, q/e=旋转, space=停止')
        
    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.current_vx
        msg.linear.y = self.current_vy
        msg.angular.z = self.current_wz
        self.publisher.publish(msg)
        
        self.get_logger().info(
            f'发布速度: vx={self.current_vx:.2f}, vy={self.current_vy:.2f}, wz={self.current_wz:.2f}'
        )


def get_key():
    """获取键盘输入（非阻塞）"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = TestVelocityPublisher()
    
    try:
        while rclpy.ok():
            key = get_key()
            
            if key == 'w':
                node.current_vx += node.linear_step
            elif key == 's':
                node.current_vx -= node.linear_step
            elif key == 'a':
                node.current_vy += node.linear_step
            elif key == 'd':
                node.current_vy -= node.linear_step
            elif key == 'q':
                node.current_wz += node.angular_step
            elif key == 'e':
                node.current_wz -= node.angular_step
            elif key == ' ':
                node.current_vx = 0.0
                node.current_vy = 0.0
                node.current_wz = 0.0
            elif key == '\x03':  # Ctrl+C
                break
            else:
                continue
            
            node.publish_velocity()
            rclpy.spin_once(node, timeout_sec=0.01)
            
    except KeyboardInterrupt:
        pass
    finally:
        # 停止机器人
        node.current_vx = 0.0
        node.current_vy = 0.0
        node.current_wz = 0.0
        node.publish_velocity()
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
