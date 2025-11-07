#!/usr/bin/env python3
"""
测试扫描过滤效果
同时可视化原始扫描和过滤后的扫描
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanComparer(Node):
    def __init__(self):
        super().__init__('scan_comparer')
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.filtered_sub = self.create_subscription(
            LaserScan, '/scan_filtered', self.filtered_callback, 10)
        
        self.scan_data = None
        self.filtered_data = None
        
        self.timer = self.create_timer(2.0, self.compare)
        
        self.get_logger().info('扫描对比节点已启动')
        self.get_logger().info('订阅: /scan 和 /scan_filtered')
    
    def scan_callback(self, msg):
        self.scan_data = msg
    
    def filtered_callback(self, msg):
        self.filtered_data = msg
    
    def compare(self):
        if self.scan_data is None or self.filtered_data is None:
            self.get_logger().warn('等待扫描数据...')
            return
        
        # 统计有效点数
        valid_original = sum(1 for r in self.scan_data.ranges 
                           if not math.isinf(r) and not math.isnan(r))
        valid_filtered = sum(1 for r in self.filtered_data.ranges 
                           if not math.isinf(r) and not math.isnan(r))
        
        filtered_count = valid_original - valid_filtered
        filter_percent = (filtered_count / valid_original * 100) if valid_original > 0 else 0
        
        self.get_logger().info('─' * 60)
        self.get_logger().info(f'原始扫描: {valid_original} 个有效点')
        self.get_logger().info(f'过滤后:   {valid_filtered} 个有效点')
        self.get_logger().info(f'已过滤:   {filtered_count} 个点 ({filter_percent:.1f}%)')
        self.get_logger().info('─' * 60)

def main(args=None):
    rclpy.init(args=args)
    node = ScanComparer()
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
