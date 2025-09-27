#!/usr/bin/env python3
"""
简化的Franka Emika FR3 末端执行器控制脚本

这是一个简化版本，专门用于控制Franka机械臂运动到指定的末端执行器位置。
使用MoveIt2的Python接口，支持笛卡尔空间运动。

使用方法:
1. 启动机械臂和MoveIt:
   ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=<机械臂IP地址>
2. 运行此脚本:
   python3 simple_franka_controller.py

作者: AI Assistant
日期: 2024
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_commander import MoveGroupCommander, RobotCommander
import math


class SimpleFrankaController:
    """简化的Franka控制器"""
    
    def __init__(self):
        """初始化控制器"""
        print("正在初始化Franka控制器...")
        
        # 初始化MoveIt Commander
        self.robot = RobotCommander()
        self.move_group = MoveGroupCommander('panda_manipulator')  # 或使用 'panda_arm'
        
        # 设置规划参数
        self.move_group.set_planning_time(5.0)
        self.move_group.set_num_planning_attempts(5)
        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_max_acceleration_scaling_factor(0.3)
        
        print(f"使用规划组: {self.move_group.get_name()}")
        print(f"规划框架: {self.move_group.get_planning_frame()}")
        print(f"末端执行器链接: {self.move_group.get_end_effector_link()}")
        print("控制器初始化完成!")

    def create_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        创建目标位姿
        
        Args:
            x, y, z: 位置坐标 (米)
            roll, pitch, yaw: 欧拉角 (弧度)
        
        Returns:
            geometry_msgs.msg.Pose: 目标位姿
        """
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        
        # 将欧拉角转换为四元数
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        return pose

    def move_to_position(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        移动到指定位置
        
        Args:
            x, y, z: 目标位置 (米)
            roll, pitch, yaw: 目标方向 (弧度)
        
        Returns:
            bool: 运动是否成功
        """
        print(f"\n=== 移动到位置: x={x:.3f}, y={y:.3f}, z={z:.3f} ===")
        
        # 创建目标位姿
        target_pose = self.create_pose(x, y, z, roll, pitch, yaw)
        
        # 设置目标位姿
        self.move_group.set_pose_target(target_pose)
        
        # 规划并执行运动
        print("正在规划路径...")
        success = self.move_group.go(wait=True)
        
        if success:
            print("运动执行成功!")
            # 清除目标
            self.move_group.clear_pose_targets()
            return True
        else:
            print("运动执行失败!")
            return False

    def get_current_position(self):
        """获取当前末端执行器位置"""
        current_pose = self.move_group.get_current_pose().pose
        return {
            'position': [current_pose.position.x, current_pose.position.y, current_pose.position.z],
            'orientation': [current_pose.orientation.x, current_pose.orientation.y, 
                          current_pose.orientation.z, current_pose.orientation.w]
        }

    def print_current_position(self):
        """打印当前末端执行器位置"""
        pos = self.get_current_position()
        print(f"\n当前末端执行器位置:")
        print(f"  位置: x={pos['position'][0]:.4f}, y={pos['position'][1]:.4f}, z={pos['position'][2]:.4f}")
        print(f"  方向: x={pos['orientation'][0]:.4f}, y={pos['orientation'][1]:.4f}, "
              f"z={pos['orientation'][2]:.4f}, w={pos['orientation'][3]:.4f}")


def main():
    """主函数"""
    print("=== Franka末端执行器位置控制器 ===")
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        # 创建控制器
        controller = SimpleFrankaController()
        
        # 显示当前位置
        controller.print_current_position()
        
        # 示例运动序列
        print("\n开始示例运动序列...")
        
        # 运动1: 移动到指定位置
        print("\n运动1: 移动到位置 (0.4, 0.0, 0.6)")
        if controller.move_to_position(0.4, 0.0, 0.6):
            controller.print_current_position()
        
        # 运动2: 移动到另一个位置
        print("\n运动2: 移动到位置 (0.5, 0.2, 0.5)")
        if controller.move_to_position(0.5, 0.2, 0.5):
            controller.print_current_position()
        
        # 运动3: 移动到第三个位置
        print("\n运动3: 移动到位置 (0.3, -0.2, 0.7)")
        if controller.move_to_position(0.3, -0.2, 0.7):
            controller.print_current_position()
        
        print("\n=== 运动序列完成 ===")
        
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"\n程序运行出错: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
