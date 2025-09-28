#!/usr/bin/env python3
"""
Franka FR3 笛卡尔空间控制器

这个脚本演示如何直接控制末端执行器的位姿，而不是通过关节角度。
支持多种控制模式：位置、速度、力矩控制。

使用方法:
1. 启动机械臂和控制器:
   ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=<IP> use_fake_hardware:=true fake_sensor_commands:=true
2. 运行此脚本:
   python3 simple_franka_cartesian_controller.py

作者: AI Assistant
日期: 2024
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64MultiArray
import math
import time


class FrankaCartesianController(Node):
    """Franka笛卡尔空间控制器"""
    
    def __init__(self):
        """初始化控制器"""
        super().__init__('franka_cartesian_controller')
        print("正在初始化Franka笛卡尔控制器...")
        
        # 发布器 - 不同的控制接口
        self.pose_publisher = self.create_publisher(
            PoseStamped, 
            '/fr3_cartesian_pose_controller/target_pose', 
            10
        )
        
        self.velocity_publisher = self.create_publisher(
            TwistStamped, 
            '/fr3_cartesian_velocity_controller/target_twist', 
            10
        )
        
        self.effort_publisher = self.create_publisher(
            Float64MultiArray, 
            '/fr3_effort_controller/commands', 
            10
        )
        
        print("笛卡尔控制器初始化完成!")
        print("可用控制模式:")
        print("1. 位姿控制 (Pose Control)")
        print("2. 速度控制 (Velocity Control)")  
        print("3. 力矩控制 (Effort Control)")

    def move_to_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        移动到指定的笛卡尔位姿
        
        Args:
            x, y, z: 位置坐标 (米)
            roll, pitch, yaw: 欧拉角 (弧度)
        """
        print(f"\n=== 笛卡尔位姿控制: 移动到 x={x:.3f}, y={y:.3f}, z={z:.3f} ===")
        
        # 创建位姿消息
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "fr3_link0"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 设置位置
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        
        # 将欧拉角转换为四元数
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        # 发布位姿命令
        self.pose_publisher.publish(pose_msg)
        print("位姿命令已发送")

    def move_with_velocity(self, vx, vy, vz, wx=0.0, wy=0.0, wz=0.0, duration=2.0):
        """
        以指定速度移动
        
        Args:
            vx, vy, vz: 线速度 (m/s)
            wx, wy, wz: 角速度 (rad/s)
            duration: 持续时间 (秒)
        """
        print(f"\n=== 笛卡尔速度控制: vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f} ===")
        
        # 创建速度消息
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = "fr3_link0"
        
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = vy
        twist_msg.twist.linear.z = vz
        twist_msg.twist.angular.x = wx
        twist_msg.twist.angular.y = wy
        twist_msg.twist.angular.z = wz
        
        # 发送速度命令
        start_time = time.time()
        while time.time() - start_time < duration:
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.velocity_publisher.publish(twist_msg)
            time.sleep(0.01)  # 100Hz
        
        # 停止运动
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        self.velocity_publisher.publish(twist_msg)
        
        print("速度控制完成")

    def apply_joint_efforts(self, efforts, duration=2.0):
        """
        施加关节力矩
        
        Args:
            efforts: 7个关节的力矩值列表 (N⋅m)
            duration: 持续时间 (秒)
        """
        print(f"\n=== 关节力矩控制: {[f'{e:.2f}' for e in efforts]} ===")
        
        if len(efforts) != 7:
            print("错误：必须提供7个关节的力矩值")
            return
        
        # 创建力矩消息
        effort_msg = Float64MultiArray()
        effort_msg.data = efforts
        
        # 发送力矩命令
        start_time = time.time()
        while time.time() - start_time < duration:
            self.effort_publisher.publish(effort_msg)
            time.sleep(0.01)  # 100Hz
        
        # 停止施加力矩
        effort_msg.data = [0.0] * 7
        self.effort_publisher.publish(effort_msg)
        
        print("力矩控制完成")


def main():
    """主函数"""
    print("=== Franka笛卡尔空间控制器 ===")
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        # 创建控制器
        controller = FrankaCartesianController()
        
        print("\n开始笛卡尔控制演示...")
        
        # 演示1: 位姿控制
        print("\n--- 演示1: 位姿控制 ---")
        controller.move_to_pose(0.4, 0.0, 0.6)
        time.sleep(3)
        
        controller.move_to_pose(0.5, 0.2, 0.5)
        time.sleep(3)
        
        # 演示2: 速度控制
        print("\n--- 演示2: 速度控制 ---")
        controller.move_with_velocity(0.1, 0.0, 0.0, duration=2.0)  # X方向移动
        time.sleep(1)
        
        controller.move_with_velocity(0.0, 0.1, 0.0, duration=2.0)  # Y方向移动
        time.sleep(1)
        
        # 演示3: 力矩控制（小力矩，安全）
        print("\n--- 演示3: 力矩控制 ---")
        small_efforts = [0.1, 0.1, 0.0, 0.1, 0.0, 0.0, 0.0]  # 小力矩值
        controller.apply_joint_efforts(small_efforts, duration=1.0)
        
        print("\n=== 笛卡尔控制演示完成 ===")
        
        # 销毁节点
        controller.destroy_node()
        
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"\n程序运行出错: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
