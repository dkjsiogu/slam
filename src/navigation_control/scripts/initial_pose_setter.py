#!/usr/bin/env python3
"""
自动设置初始位姿节点
在地图中央设置机器人初始位姿，触发 AMCL 开始定位
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import time

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('delay', 2.0)  # 延迟秒数，等待其他节点启动
        
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value
        self.initial_yaw = self.get_parameter('initial_yaw').value
        self.delay = self.get_parameter('delay').value
        
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # 订阅地图以获取地图信息
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
        
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
        
        self.map_received = False
        self.pose_published = False
        
        self.get_logger().info('初始位姿设置节点已启动')
        self.get_logger().info(f'将在 {self.delay} 秒后发布初始位姿')
    
    def map_callback(self, msg):
        if not self.map_received:
            self.map_received = True
            self.get_logger().info(f'收到地图: {msg.info.width}x{msg.info.height}')
            
            # 延迟后发布初始位姿
            self.timer = self.create_timer(self.delay, self.publish_initial_pose)
    
    def publish_initial_pose(self):
        if self.pose_published:
            return
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # 设置位置
        pose_msg.pose.pose.position.x = self.initial_x
        pose_msg.pose.pose.position.y = self.initial_y
        pose_msg.pose.pose.position.z = 0.0
        
        # 设置朝向 (从 yaw 角度转为四元数)
        import math
        yaw = self.initial_yaw
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # 设置协方差 (表示初始位姿的不确定性)
        # 增大协方差以允许 AMCL 更大范围搜索粒子
        pose_msg.pose.covariance = [0.0] * 36
        pose_msg.pose.covariance[0] = 0.5   # x 方向方差 (增大到 0.5)
        pose_msg.pose.covariance[7] = 0.5   # y 方向方差 (增大到 0.5)
        pose_msg.pose.covariance[35] = 0.2  # yaw 方向方差 (增大到 0.2)
        
        # 发布多次确保收到
        for _ in range(5):
            self.pose_pub.publish(pose_msg)
            time.sleep(0.1)
        
        self.pose_published = True
        self.get_logger().info(f'✓ 已发布初始位姿: x={self.initial_x:.2f}, y={self.initial_y:.2f}, yaw={self.initial_yaw:.2f}')
        
        # 停止定时器
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
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
