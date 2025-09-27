#!/usr/bin/env python3
"""
Franka直接控制脚本 (不依赖MoveIt)

这个脚本使用Franka的基础控制接口，不依赖MoveIt，用于解决MoveIt启动问题。

使用方法:
1. 启动基础Franka控制:
   ros2 launch franka_bringup franka.launch.py robot_ip:=192.168.2.2
2. 运行此脚本:
   python3 franka_direct_control.py

作者: AI Assistant
日期: 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math
import time


class FrankaDirectControl(Node):
    """Franka直接控制器"""
    
    def __init__(self):
        super().__init__('franka_direct_control')
        
        # 创建轨迹跟踪action客户端
        self.trajectory_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/fr3_arm_controller/follow_joint_trajectory'
        )
        
        # 等待action服务器
        self.get_logger().info('等待轨迹跟踪服务器...')
        self.trajectory_client.wait_for_server()
        self.get_logger().info('轨迹跟踪服务器已连接!')
        
        # 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/franka/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 关节名称
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        # 当前关节位置
        self.current_joint_positions = [0.0] * 7
        self.joint_states_received = False
        
        self.get_logger().info('Franka直接控制器初始化完成!')

    def joint_state_callback(self, msg):
        """关节状态回调函数"""
        if len(msg.position) >= 7:
            self.current_joint_positions = list(msg.position[:7])
            self.joint_states_received = True

    def wait_for_joint_states(self, timeout=5.0):
        """等待接收关节状态"""
        start_time = time.time()
        while not self.joint_states_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.joint_states_received:
            self.get_logger().info('已接收关节状态')
            return True
        else:
            self.get_logger().error('未接收到关节状态')
            return False

    def print_current_joint_positions(self):
        """打印当前关节位置"""
        self.get_logger().info("当前关节位置:")
        for i, (name, pos) in enumerate(zip(self.joint_names, self.current_joint_positions)):
            self.get_logger().info(f"  {name}: {pos:.4f} rad ({math.degrees(pos):.2f}°)")

    def create_trajectory_goal(self, target_positions, duration=5.0):
        """创建轨迹目标"""
        goal = FollowJointTrajectory.Goal()
        
        # 设置轨迹
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # 创建轨迹点
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        trajectory.points = [point]
        goal.trajectory = trajectory
        
        return goal

    def move_to_joint_positions(self, target_positions, duration=5.0):
        """移动到指定关节位置"""
        self.get_logger().info("=== 开始关节运动 ===")
        self.print_current_joint_positions()
        
        self.get_logger().info("目标关节位置:")
        for i, (name, pos) in enumerate(zip(self.joint_names, target_positions)):
            self.get_logger().info(f"  {name}: {pos:.4f} rad ({math.degrees(pos):.2f}°)")
        
        # 创建轨迹目标
        goal = self.create_trajectory_goal(target_positions, duration)
        
        # 发送目标
        future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('目标被接受，等待结果...')
                
                # 等待动作完成
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                
                result = result_future.result()
                if result.status == result.status.SUCCEEDED:
                    self.get_logger().info('运动执行成功!')
                    return True
                else:
                    self.get_logger().error(f'运动执行失败: {result.status}')
                    return False
            else:
                self.get_logger().error('目标被拒绝')
                return False
        else:
            self.get_logger().error('动作服务调用失败')
            return False

    def move_to_home_position(self):
        """移动到home位置"""
        home_positions = [0.0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4]
        return self.move_to_joint_positions(home_positions)

    def move_to_ready_position(self):
        """移动到ready位置"""
        ready_positions = [0.0, -math.pi/6, 0.0, -2*math.pi/3, 0.0, math.pi/2, 0.0]
        return self.move_to_joint_positions(ready_positions)

    def create_safe_motion_sequence(self):
        """创建安全的运动序列"""
        sequences = [
            {
                'name': 'Home位置',
                'positions': [0.0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4]
            },
            {
                'name': 'Ready位置',
                'positions': [0.0, -math.pi/6, 0.0, -2*math.pi/3, 0.0, math.pi/2, 0.0]
            },
            {
                'name': '左偏位置',
                'positions': [math.pi/6, -math.pi/6, 0.0, -2*math.pi/3, 0.0, math.pi/2, 0.0]
            },
            {
                'name': '右偏位置',
                'positions': [-math.pi/6, -math.pi/6, 0.0, -2*math.pi/3, 0.0, math.pi/2, 0.0]
            }
        ]
        
        return sequences


def main():
    """主函数"""
    rclpy.init()
    
    try:
        # 创建控制器
        controller = FrankaDirectControl()
        
        # 等待关节状态
        if not controller.wait_for_joint_states():
            controller.get_logger().error("无法获取关节状态，退出")
            return
        
        # 显示当前状态
        controller.print_current_joint_positions()
        
        # 创建安全运动序列
        sequences = controller.create_safe_motion_sequence()
        
        controller.get_logger().info("\n=== 开始安全运动序列 ===")
        
        for i, seq in enumerate(sequences):
            controller.get_logger().info(f"\n运动 {i+1}: {seq['name']}")
            
            if controller.move_to_joint_positions(seq['positions'], duration=3.0):
                controller.get_logger().info(f"✅ {seq['name']} 完成")
                time.sleep(1)
            else:
                controller.get_logger().error(f"❌ {seq['name']} 失败")
                break
        
        controller.get_logger().info("\n=== 运动序列完成 ===")
        
        # 返回home位置
        controller.get_logger().info("\n返回Home位置...")
        controller.move_to_home_position()
        
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"\n程序运行出错: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
