#!/usr/bin/env python3
"""
简单目标点控制器
订阅 RViz2 的 2D Nav Goal，计算全向轮需要的 vx, vy 速度
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math

class SimpleGoalController(Node):
    def __init__(self):
        super().__init__('simple_goal_controller')
        
        # 参数
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('lookahead_distance', 0.3)
        
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        
        # 订阅目标点 (RViz2 2D Nav Goal)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # 订阅机器人位姿 (AMCL)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 定时器 - 控制循环
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.current_pose = None
        self.goal_pose = None
        
        self.get_logger().info('简单导航控制器已启动')
        self.get_logger().info(f'最大速度: {self.max_linear_vel} m/s')
        self.get_logger().info(f'目标容差: {self.goal_tolerance} m')
    
    def pose_callback(self, msg):
        """接收机器人当前位姿"""
        self.current_pose = msg.pose.pose
    
    def goal_callback(self, msg):
        """接收目标点"""
        self.goal_pose = msg.pose
        self.get_logger().info(f'新目标: x={self.goal_pose.position.x:.2f}, y={self.goal_pose.position.y:.2f}')
    
    def control_loop(self):
        """控制循环 - 计算并发布 vx, vy"""
        if self.current_pose is None or self.goal_pose is None:
            return
        
        # 计算到目标的距离和方向
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # 到达目标
        if distance < self.goal_tolerance:
            self.get_logger().info('到达目标！', throttle_duration_sec=1.0)
            self.publish_velocity(0.0, 0.0, 0.0)
            self.goal_pose = None
            return
        
        # 获取机器人当前朝向
        q = self.current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # 计算目标方向角
        goal_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(goal_angle - robot_yaw)
        
        # 计算机器人坐标系下的速度
        # vx: 前后方向 (机器人坐标系)
        # vy: 左右方向 (机器人坐标系)
        speed = min(distance, self.max_linear_vel)
        
        vx_global = speed * math.cos(goal_angle)
        vy_global = speed * math.sin(goal_angle)
        
        # 转换到机器人坐标系
        vx = vx_global * math.cos(-robot_yaw) - vy_global * math.sin(-robot_yaw)
        vy = vx_global * math.sin(-robot_yaw) + vy_global * math.cos(-robot_yaw)
        
        # 计算旋转速度 (朝向目标方向)
        wz = max(-self.max_angular_vel, min(self.max_angular_vel, 2.0 * angle_diff))
        
        # 如果角度偏差大，减小线速度
        if abs(angle_diff) > 0.5:
            vx *= 0.5
            vy *= 0.5
        
        self.publish_velocity(vx, vy, wz)
        
        self.get_logger().info(
            f'距离:{distance:.2f}m, vx:{vx:.2f}, vy:{vy:.2f}, wz:{wz:.2f}',
            throttle_duration_sec=0.5
        )
    
    def publish_velocity(self, vx, vy, wz):
        """发布速度命令"""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.cmd_vel_pub.publish(msg)
    
    def normalize_angle(self, angle):
        """归一化角度到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGoalController()
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
