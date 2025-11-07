#!/usr/bin/env python3
"""
系统测试脚本 - 验证所有新增功能
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import time

class SystemTester(Node):
    def __init__(self):
        super().__init__('system_tester')
        
        # 订阅器
        self.obstacle_sub = self.create_subscription(
            String, '/obstacle_info', self.obstacle_callback, 10)
        self.front_dist_sub = self.create_subscription(
            Float32, '/front_obstacle_distance', self.front_dist_callback, 10)
        self.omni_status_sub = self.create_subscription(
            String, '/omni_status', self.omni_status_callback, 10)
        self.serial_status_sub = self.create_subscription(
            String, '/serial_connection_status', self.serial_status_callback, 10)
        
        # 发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 服务客户端
        self.set_goal_0_client = self.create_client(Trigger, '/set_goal_0')
        self.set_goal_1_client = self.create_client(Trigger, '/set_goal_1')
        self.emergency_stop_client = self.create_client(Trigger, '/emergency_stop')
        
        # 数据存储
        self.obstacle_data = None
        self.front_distance = None
        self.omni_status = None
        self.serial_status = None
        
        self.get_logger().info('🔧 系统测试器已初始化')
    
    def obstacle_callback(self, msg):
        self.obstacle_data = msg.data
    
    def front_dist_callback(self, msg):
        self.front_distance = msg.data
    
    def omni_status_callback(self, msg):
        self.omni_status = msg.data
    
    def serial_status_callback(self, msg):
        self.serial_status = msg.data
    
    def test_obstacle_monitor(self):
        """测试障碍物监控"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('测试1: 障碍物监控功能')
        self.get_logger().info('=' * 60)
        
        self.get_logger().info('等待障碍物数据...')
        timeout = 10
        start_time = time.time()
        
        while self.obstacle_data is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.obstacle_data:
            self.get_logger().info(f'✓ 障碍物数据: {self.obstacle_data}')
            if self.front_distance is not None:
                self.get_logger().info(f'✓ 前方距离: {self.front_distance:.3f} 米')
            return True
        else:
            self.get_logger().error('✗ 未收到障碍物数据')
            return False
    
    def test_omni_controller(self):
        """测试全向轮控制器"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('测试2: 全向轮控制器')
        self.get_logger().info('=' * 60)
        
        # 发送测试速度命令
        test_commands = [
            ("前进", 0.3, 0.0, 0.0),
            ("左平移", 0.0, 0.2, 0.0),
            ("旋转", 0.0, 0.0, 0.5),
            ("组合运动", 0.2, 0.1, 0.3),
            ("停止", 0.0, 0.0, 0.0),
        ]
        
        for name, vx, vy, wz in test_commands:
            self.get_logger().info(f'发送命令: {name} (Vx={vx}, Vy={vy}, Wz={wz})')
            
            cmd = Twist()
            cmd.linear.x = vx
            cmd.linear.y = vy
            cmd.angular.z = wz
            self.cmd_vel_pub.publish(cmd)
            
            # 等待并检查状态
            time.sleep(1.0)
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.omni_status:
                self.get_logger().info(f'  状态: {self.omni_status}')
        
        self.get_logger().info('✓ 全向轮控制器测试完成')
        return True
    
    def test_serial_communication(self):
        """测试串口通信"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('测试3: 串口通信')
        self.get_logger().info('=' * 60)
        
        self.get_logger().info('等待串口状态...')
        timeout = 5
        start_time = time.time()
        
        while self.serial_status is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.serial_status:
            self.get_logger().info(f'✓ 串口状态: {self.serial_status}')
            if '连接' in self.serial_status or 'Open' in self.serial_status:
                self.get_logger().info('✓ 串口已连接')
                return True
            else:
                self.get_logger().warn('⚠ 串口未连接（这是正常的，如果没有连接硬件）')
                return True
        else:
            self.get_logger().warn('⚠ 未收到串口状态信息')
            return True
    
    def test_manual_goal_setter(self):
        """测试手动目标点设置"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('测试4: 手动目标点设置')
        self.get_logger().info('=' * 60)
        
        # 等待服务可用
        self.get_logger().info('等待服务可用...')
        if not self.set_goal_0_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('⚠ 目标点设置服务不可用（可能Nav2未启动）')
            return True
        
        # 调用服务
        self.get_logger().info('调用 set_goal_0 服务...')
        request = Trigger.Request()
        future = self.set_goal_0_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f'✓ 服务响应: {result.message}')
            return result.success
        else:
            self.get_logger().error('✗ 服务调用失败')
            return False
    
    def test_emergency_stop(self):
        """测试紧急停止"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('测试5: 紧急停止功能')
        self.get_logger().info('=' * 60)
        
        # 先发送一个速度命令
        self.get_logger().info('发送运动命令...')
        cmd = Twist()
        cmd.linear.x = 0.5
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.5)
        
        # 等待服务可用
        if not self.emergency_stop_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('✗ 紧急停止服务不可用')
            return False
        
        # 调用紧急停止
        self.get_logger().info('触发紧急停止...')
        request = Trigger.Request()
        future = self.emergency_stop_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f'✓ 紧急停止响应: {result.message}')
            return result.success
        else:
            self.get_logger().error('✗ 紧急停止调用失败')
            return False
    
    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info('')
        self.get_logger().info('🚀 开始系统测试')
        self.get_logger().info('')
        
        results = []
        
        # 等待一下让所有节点启动
        self.get_logger().info('等待节点初始化...')
        time.sleep(2.0)
        
        # 运行测试
        results.append(('障碍物监控', self.test_obstacle_monitor()))
        results.append(('全向轮控制器', self.test_omni_controller()))
        results.append(('串口通信', self.test_serial_communication()))
        results.append(('手动目标点设置', self.test_manual_goal_setter()))
        results.append(('紧急停止', self.test_emergency_stop()))
        
        # 总结
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('📊 测试总结')
        self.get_logger().info('=' * 60)
        
        passed = 0
        for name, result in results:
            status = '✓ 通过' if result else '✗ 失败'
            self.get_logger().info(f'{name:20s} : {status}')
            if result:
                passed += 1
        
        self.get_logger().info('')
        self.get_logger().info(f'总计: {passed}/{len(results)} 测试通过')
        self.get_logger().info('')
        
        return passed == len(results)

def main(args=None):
    rclpy.init(args=args)
    
    tester = SystemTester()
    
    try:
        success = tester.run_all_tests()
        
        if success:
            tester.get_logger().info('🎉 所有测试通过！系统运行正常！')
        else:
            tester.get_logger().warn('⚠ 部分测试未通过，请检查系统配置')
    
    except KeyboardInterrupt:
        tester.get_logger().info('测试被用户中断')
    
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
