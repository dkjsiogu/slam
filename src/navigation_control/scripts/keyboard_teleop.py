#!/usr/bin/env python3
"""
键盘遥控节点 - 用于建图时手动控制机器人
支持 WASD 控制 + 平滑速度过渡

按键映射:
  W - 前进 (0.5 m/s)
  S - 后退 (-0.5 m/s)
  A - 左移 (0.5 m/s)
  D - 右移 (-0.5 m/s)
  Q - 左转 (1.0 rad/s)
  E - 右转 (-1.0 rad/s)
  Space - 急停
  ESC - 退出

使用方法:
  ros2 run navigation_control keyboard_teleop.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # 速度发布器
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 控制参数
        self.linear_speed = 0.5   # m/s
        self.angular_speed = 1.0  # rad/s
        
        # 当前速度
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        
        # 定时器 - 10Hz 发布
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        # 保存终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('═' * 60)
        self.get_logger().info('键盘遥控节点已启动')
        self.get_logger().info('═' * 60)
        self.get_logger().info('控制说明:')
        self.get_logger().info('  W/S - 前进/后退 (±0.5 m/s)')
        self.get_logger().info('  A/D - 左移/右移 (±0.5 m/s)')
        self.get_logger().info('  Q/E - 左转/右转 (±1.0 rad/s)')
        self.get_logger().info('  Space - 急停')
        self.get_logger().info('  ESC - 退出')
        self.get_logger().info('═' * 60)
        
    def get_key(self):
        """非阻塞获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_velocity(self):
        """发布速度命令"""
        # 获取按键
        key = self.get_key()
        
        # 松开按键后速度归零（除非有新按键）
        if key == '':
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_wz = 0.0
        
        # 处理按键
        if key == 'w' or key == 'W':
            self.current_vx = self.linear_speed
            self.current_vy = 0.0
            self.current_wz = 0.0
            self.get_logger().info('↑ 前进')
            
        elif key == 's' or key == 'S':
            self.current_vx = -self.linear_speed
            self.current_vy = 0.0
            self.current_wz = 0.0
            self.get_logger().info('↓ 后退')
            
        elif key == 'a' or key == 'A':
            self.current_vx = 0.0
            self.current_vy = self.linear_speed
            self.current_wz = 0.0
            self.get_logger().info('← 左移')
            
        elif key == 'd' or key == 'D':
            self.current_vx = 0.0
            self.current_vy = -self.linear_speed
            self.current_wz = 0.0
            self.get_logger().info('→ 右移')
            
        elif key == 'q' or key == 'Q':
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_wz = self.angular_speed
            self.get_logger().info('↺ 左转')
            
        elif key == 'e' or key == 'E':
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_wz = -self.angular_speed
            self.get_logger().info('↻ 右转')
            
        elif key == ' ':
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_wz = 0.0
            self.get_logger().info('■ 急停')
            
        elif key == '\x1b':  # ESC
            self.get_logger().info('退出键盘控制')
            rclpy.shutdown()
            return
        
        # 发布速度
        msg = Twist()
        msg.linear.x = self.current_vx
        msg.linear.y = self.current_vy
        msg.angular.z = self.current_wz
        self.vel_pub.publish(msg)
    
    def __del__(self):
        """恢复终端设置"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止机器人
        msg = Twist()
        node.vel_pub.publish(msg)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
