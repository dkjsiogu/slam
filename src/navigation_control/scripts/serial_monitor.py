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
        """显示解析后的里程计数据"""
        if len(msg.data) >= 9:
            # 机器人坐标系增量
            delta_x_robot = msg.data[0]
            delta_y_robot = msg.data[1]
            delta_theta = msg.data[2]
            dt = msg.data[3]
            
            # 世界坐标系增量
            delta_x_world = msg.data[4]
            delta_y_world = msg.data[5]
            
            # 累计位姿
            pose_x = msg.data[6]
            pose_y = msg.data[7]
            pose_theta = msg.data[8]
            
            # 计算瞬时速度
            if dt > 0.001:
                vx = delta_x_world / dt
                vy = delta_y_world / dt
                wz = delta_theta / dt
                
                self.get_logger().info('─' * 80)
                self.get_logger().info(f'[下位机里程计数据 - 50ms窗口积分]')
                self.get_logger().info(f'  机器人坐标系增量: dx={delta_x_robot:7.4f}m  dy={delta_y_robot:7.4f}m  dθ={delta_theta:7.4f}rad')
                self.get_logger().info(f'  世界坐标系增量:   dx={delta_x_world:7.4f}m  dy={delta_y_world:7.4f}m')
                self.get_logger().info(f'  累计位姿:         x={pose_x:7.3f}m   y={pose_y:7.3f}m   θ={pose_theta:7.3f}rad ({pose_theta*57.3:.1f}°)')
                self.get_logger().info(f'  时间增量:         dt={dt:7.4f}s ({dt*1000:.1f}ms)')
                self.get_logger().info(f'  估算速度:         vx={vx:7.3f}m/s  vy={vy:7.3f}m/s  wz={wz:7.3f}rad/s')
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
