#!/usr/bin/env python3
"""
简单目标点控制器
订阅 RViz2 的 2D Nav Goal，计算全向轮需要的 vx, vy 速度
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class SimpleGoalController(Node):
    def __init__(self):
        super().__init__('simple_goal_controller')
        
        # 参数
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('goal_tolerance', 0.05)  # 降低到5cm，更精确
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
        
        # 订阅机器人位姿 (优先 AMCL，备选 odom)
        self.pose_sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )
        
        # 订阅里程计 (作为备选定位源，适用于 Cartographer)
        self.pose_sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 发布规划路径（用于 RViz2 可视化）
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # TF监听器 - 用于获取 map -> base_link 变换
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 定时器 - 控制循环
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.current_pose = None
        self.goal_pose = None
        self.use_amcl = False  # 标记是否使用 AMCL
        self.initial_yaw = None  # 记录出发时的 yaw 角度
        
        self.get_logger().info('简单导航控制器已启动')
        self.get_logger().info(f'最大速度: {self.max_linear_vel} m/s')
        self.get_logger().info(f'目标容差: {self.goal_tolerance} m')
        self.get_logger().info('策略: 保持初始朝向 + 全向轮移动')
    
    def amcl_pose_callback(self, msg):
        """接收 AMCL 位姿 (优先使用)"""
        self.current_pose = msg.pose.pose
        if not self.use_amcl:
            self.use_amcl = True
            self.get_logger().info('使用 AMCL 定位')
    
    def odom_callback(self, msg):
        """接收里程计位姿 (Cartographer 模式下使用)"""
        if not self.use_amcl:  # 只在没有 AMCL 时使用
            self.current_pose = msg.pose.pose
    
    def goal_callback(self, msg):
        """接收目标点（在 map 坐标系）"""
        self.goal_pose = msg.pose
        
        # 记录出发时的朝向 (用于保持朝向不变)
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.initial_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            self.get_logger().info(f'新目标(map): x={self.goal_pose.position.x:.2f}, y={self.goal_pose.position.y:.2f}')
            self.get_logger().info(f'锁定初始朝向: {math.degrees(self.initial_yaw):.1f}°')
            
            # 发布规划路径（直线）
            self.publish_path()
        except TransformException as ex:
            self.get_logger().error(f'无法获取当前位姿: {ex}')
    
    def control_loop(self):
        """控制循环 - 计算并发布 vx, vy"""
        if self.goal_pose is None:
            return
        
        # 使用 TF 获取机器人在 map 坐标系下的位姿
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # 从TF获取当前位置和朝向
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            
        except TransformException as ex:
            self.get_logger().warn(f'无法获取 TF: {ex}', throttle_duration_sec=1.0)
            return
        
        # 计算到目标的距离和方向（都在 map 坐标系）
        dx = self.goal_pose.position.x - current_x
        dy = self.goal_pose.position.y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # 到达目标
        if distance < self.goal_tolerance:
            self.get_logger().info('到达目标！', throttle_duration_sec=1.0)
            self.publish_velocity(0.0, 0.0, 0.0)
            self.goal_pose = None
            return
        
        # 全向轮策略：保持初始朝向 + vx/vy 移动到目标
        # 利用 IMU 高精度 yaw 值，不旋转机器人
        
        # 速度规划：距离远时加速，接近时减速
        if distance > 0.5:
            speed = self.max_linear_vel  # 远距离：全速
        elif distance > 0.2:
            speed = self.max_linear_vel * 0.6  # 中距离：减速到60%
        else:
            speed = min(distance * 1.5, self.max_linear_vel * 0.3)  # 近距离：线性减速
        
        # 世界坐标系 → 机器人坐标系转换
        # 目标在世界坐标系的相对位置: (dx, dy)
        # 机器人朝向: robot_yaw
        # 转换到机器人坐标系（X前 Y左）:
        #   dx_robot = dx*cos(yaw) + dy*sin(yaw)  （前后方向）
        #   dy_robot = -dx*sin(yaw) + dy*cos(yaw) （左右方向）
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        dx_robot = dx * cos_yaw + dy * sin_yaw   # 机器人X轴（前+/后-）
        dy_robot = -dx * sin_yaw + dy * cos_yaw  # 机器人Y轴（左+/右-）
        
        # 归一化方向并计算速度
        dist_robot = math.sqrt(dx_robot**2 + dy_robot**2)
        if dist_robot > 0:
            vx = (dx_robot / dist_robot) * speed  # X轴速度（前后）
            vy = (dy_robot / dist_robot) * speed  # Y轴速度（左右）
        else:
            vx = 0.0
            vy = 0.0
        
        # 保持初始朝向（利用 IMU yaw 值）
        if self.initial_yaw is not None:
            yaw_error = self.normalize_angle(self.initial_yaw - robot_yaw)
            # P控制保持朝向，增益适中
            if abs(yaw_error) > 0.02:  # 死区 > 1.1°
                wz = max(-0.3, min(0.3, 1.5 * yaw_error))  # 限制最大角速度
            else:
                wz = 0.0
        else:
            wz = 0.0
        
        self.publish_velocity(vx, vy, wz)
        
        # 计算朝向偏差（相对初始朝向）
        yaw_error_deg = 0.0
        if self.initial_yaw is not None:
            yaw_error_deg = math.degrees(self.normalize_angle(robot_yaw - self.initial_yaw))
        
        self.get_logger().info(
            f'[map坐标系] 目标: ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f}) '
            f'当前: ({current_x:.2f}, {current_y:.2f}) '
            f'差值: dx={dx:.2f} dy={dy:.2f}'
        )
        self.get_logger().info(
            f'[机器人] yaw={math.degrees(robot_yaw):.0f}° | '
            f'目标位置: dx_r={dx_robot:.2f} dy_r={dy_robot:.2f} | '
            f'速度命令: vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}'
        )
    
    def publish_velocity(self, vx, vy, wz):
        """发布速度命令"""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.cmd_vel_pub.publish(msg)
    
    def publish_path(self):
        """发布规划路径（当前位置到目标的直线）"""
        if self.goal_pose is None:
            return
        
        # 使用 TF 获取当前位置
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except TransformException:
            return
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        # 起点（当前位置，从TF获取）
        start_pose = PoseStamped()
        start_pose.header = path_msg.header
        start_pose.pose.position.x = transform.transform.translation.x
        start_pose.pose.position.y = transform.transform.translation.y
        start_pose.pose.position.z = transform.transform.translation.z
        start_pose.pose.orientation = transform.transform.rotation
        path_msg.poses.append(start_pose)
        
        # 终点（目标位置）
        goal_pose_stamped = PoseStamped()
        goal_pose_stamped.header = path_msg.header
        goal_pose_stamped.pose = self.goal_pose
        path_msg.poses.append(goal_pose_stamped)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info('已发布规划路径到 /planned_path')
    
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
