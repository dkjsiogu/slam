#!/usr/bin/env python3
"""
串口数据监控节点
实时显示从下位机接收到的原始数据
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Float32MultiArray
import struct
import time

class SerialMonitor(Node):
    def __init__(self):
        super().__init__('serial_monitor')
        
        # 订阅串口原始数据
        self.raw_data_sub = self.create_subscription(
            UInt8MultiArray,
            '/serial_raw_data',
            self.raw_data_callback,
            10
        )
        
        # 订阅解析后的里程计数据
        self.odom_data_sub = self.create_subscription(
            Float32MultiArray,
            '/wheel_odom_data',
            self.odom_data_callback,
            10
        )
        
        # 统计信息
        self.packet_count = 0
        self.error_count = 0
        self.last_time = time.time()
        self.packet_rate = 0.0
        
        # 定时器 - 显示统计信息
        self.timer = self.create_timer(2.0, self.print_statistics)
        
        self.get_logger().info('串口监控节点已启动')
        self.get_logger().info('=' * 80)
    
    def raw_data_callback(self, msg):
        """显示原始字节数据"""
        self.packet_count += 1
        
        # 计算频率
        current_time = time.time()
        dt = current_time - self.last_time
        if dt > 0:
            self.packet_rate = 0.9 * self.packet_rate + 0.1 * (1.0 / dt)
        self.last_time = current_time
        
        # 显示原始数据 (每10个包显示一次)
        if self.packet_count % 10 == 0:
            hex_str = ' '.join([f'{b:02X}' for b in msg.data[:20]])  # 显示前20字节
            self.get_logger().info(f'[原始数据] {hex_str}...')
    
    def odom_data_callback(self, msg):
        """显示解析后的里程计数据（新版：基于速度积分）"""
        if len(msg.data) >= 16:
            # 机器人坐标系速度（下位机实时发送）
            vx_robot = msg.data[0]
            vy_robot = msg.data[1]
            wz_robot = msg.data[2]
            dt = msg.data[3]
            
            # 世界坐标系位移增量（上位机积分结果）
            dx_world = msg.data[4]
            dy_world = msg.data[5]
            
            # 累计位姿
            pose_x = msg.data[6]
            pose_y = msg.data[7]
            pose_theta = msg.data[8]
            
            # IMU数据
            roll = msg.data[9]
            pitch = msg.data[10]
            yaw = msg.data[11]
            
            # 角度增量
            dtheta = msg.data[12]
            
            # 世界坐标系速度（转换后）
            vx_world = msg.data[13]
            vy_world = msg.data[14]
            
            # 静止标志
            is_stationary = (msg.data[15] > 0.5)
            
            # 显示数据
            self.get_logger().info('─' * 80)
            self.get_logger().info(f'[里程计数据 - ROS时间戳积分] {"🛑 静止" if is_stationary else "▶️ 运动"}')
            self.get_logger().info(f'  机器人坐标系速度: vx={vx_robot:7.3f}m/s  vy={vy_robot:7.3f}m/s  w={wz_robot:7.3f}rad/s')
            self.get_logger().info(f'  时间增量:         dt={dt:7.4f}s ({dt*1000:.1f}ms)')
            self.get_logger().info(f'  世界坐标系增量:   dx={dx_world:7.4f}m  dy={dy_world:7.4f}m  dθ={dtheta:7.4f}rad')
            self.get_logger().info(f'  世界坐标系速度:   vx={vx_world:7.3f}m/s  vy={vy_world:7.3f}m/s')
            self.get_logger().info(f'  累计位姿:         x={pose_x:7.3f}m   y={pose_y:7.3f}m   θ={pose_theta:7.3f}rad ({pose_theta*57.3:.1f}°)')
            self.get_logger().info(f'  IMU姿态:          Roll={roll*57.3:.1f}°  Pitch={pitch*57.3:.1f}°  Yaw={yaw*57.3:.1f}°')
            self.get_logger().info('─' * 80)
    
    def print_statistics(self):
        """定期打印统计信息"""
        self.get_logger().info('')
        self.get_logger().info('═' * 80)
        self.get_logger().info(f'[统计信息]')
        self.get_logger().info(f'  总包数: {self.packet_count}')
        self.get_logger().info(f'  错误数: {self.error_count}')
        self.get_logger().info(f'  数据率: {self.packet_rate:.1f} Hz')
        self.get_logger().info(f'  成功率: {(1 - self.error_count/max(self.packet_count, 1))*100:.1f}%')
        self.get_logger().info('═' * 80)
        self.get_logger().info('')

def main(args=None):
    rclpy.init(args=args)
    node = SerialMonitor()
    
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
