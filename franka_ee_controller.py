#!/usr/bin/env python3
"""
Franka Emika FR3 末端执行器位置控制脚本

此脚本使用ROS2和MoveIt2来控制Franka机械臂运动到指定的末端执行器位置。
支持笛卡尔空间运动规划和关节空间运动规划。

使用方法:
1. 确保MoveIt2和Franka ROS2包已正确安装和配置
2. 启动机械臂和MoveIt:
   ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=<机械臂IP地址>
3. 运行此脚本:
   python3 franka_ee_controller.py

作者: AI Assistant
日期: 2024
"""

import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_commander.conversions import pose_to_list
import tf2_ros
import tf2_geometry_msgs


class FrankaEEController(Node):
    """
    Franka末端执行器控制器类
    
    提供以下功能:
    - 移动到指定的笛卡尔位置
    - 移动到指定的关节角度
    - 规划并执行轨迹
    - 获取当前末端执行器位置
    """
    
    def __init__(self):
        super().__init__('franka_ee_controller')
        
        # 初始化MoveIt Commander
        self.get_logger().info("正在初始化MoveIt Commander...")
        try:
            self.robot = RobotCommander()
            self.scene = PlanningSceneInterface()
            # 使用panda_manipulator作为新的move group (推荐)
            # 如果使用旧版本，可以改为'panda_arm'
            self.move_group = MoveGroupCommander('panda_manipulator')
            self.get_logger().info("MoveIt Commander初始化成功!")
        except Exception as e:
            self.get_logger().error(f"MoveIt Commander初始化失败: {e}")
            sys.exit(1)
            
        # 设置规划参数
        self.move_group.set_planning_time(10.0)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        
        # 获取规划组信息
        self.group_name = self.move_group.get_name()
        self.get_logger().info(f"使用规划组: {self.group_name}")
        
        # 获取机器人参考框架
        self.planning_frame = self.move_group.get_planning_frame()
        self.get_logger().info(f"规划框架: {self.planning_frame}")
        
        # 获取末端执行器链接
        self.eef_link = self.move_group.get_end_effector_link()
        self.get_logger().info(f"末端执行器链接: {self.eef_link}")
        
        # 获取关节名称
        self.joint_names = self.move_group.get_active_joints()
        self.get_logger().info(f"关节名称: {self.joint_names}")
        
        # 显示当前状态
        self.display_current_state()

    def display_current_state(self):
        """显示当前机器人状态"""
        self.get_logger().info("=== 当前机器人状态 ===")
        
        # 当前关节位置
        joint_values = self.move_group.get_current_joint_values()
        self.get_logger().info("当前关节位置:")
        for i, (name, value) in enumerate(zip(self.joint_names, joint_values)):
            self.get_logger().info(f"  {name}: {value:.4f} rad ({math.degrees(value):.2f}°)")
        
        # 当前末端执行器位置
        current_pose = self.move_group.get_current_pose().pose
        self.get_logger().info("当前末端执行器位置:")
        self.get_logger().info(f"  位置: x={current_pose.position.x:.4f}, y={current_pose.position.y:.4f}, z={current_pose.position.z:.4f}")
        self.get_logger().info(f"  方向: x={current_pose.orientation.x:.4f}, y={current_pose.orientation.y:.4f}, z={current_pose.orientation.z:.4f}, w={current_pose.orientation.w:.4f}")

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

    def move_to_pose(self, target_pose, execute=True, wait=True):
        """
        移动到指定位姿
        
        Args:
            target_pose: 目标位姿 (geometry_msgs.msg.Pose)
            execute: 是否执行运动
            wait: 是否等待运动完成
        
        Returns:
            bool: 运动是否成功
        """
        self.get_logger().info("=== 开始笛卡尔运动规划 ===")
        self.get_logger().info(f"目标位置: x={target_pose.position.x:.4f}, y={target_pose.position.y:.4f}, z={target_pose.position.z:.4f}")
        self.get_logger().info(f"目标方向: x={target_pose.orientation.x:.4f}, y={target_pose.orientation.y:.4f}, z={target_pose.orientation.z:.4f}, w={target_pose.orientation.w:.4f}")
        
        try:
            # 设置目标位姿
            self.move_group.set_pose_target(target_pose)
            
            # 规划路径
            self.get_logger().info("正在规划路径...")
            plan = self.move_group.plan()
            
            if not plan[0]:  # plan[0] 是布尔值，表示规划是否成功
                self.get_logger().error("路径规划失败!")
                return False
            
            self.get_logger().info("路径规划成功!")
            
            if execute:
                # 执行运动
                self.get_logger().info("正在执行运动...")
                success = self.move_group.execute(plan[1], wait=wait)
                if success:
                    self.get_logger().info("运动执行成功!")
                    # 清除目标
                    self.move_group.clear_pose_targets()
                    return True
                else:
                    self.get_logger().error("运动执行失败!")
                    return False
            else:
                self.get_logger().info("仅规划，不执行")
                return True
                
        except Exception as e:
            self.get_logger().error(f"运动过程中发生错误: {e}")
            return False

    def move_to_joint_positions(self, joint_positions, execute=True, wait=True):
        """
        移动到指定关节位置
        
        Args:
            joint_positions: 关节位置列表 (弧度)
            execute: 是否执行运动
            wait: 是否等待运动完成
        
        Returns:
            bool: 运动是否成功
        """
        self.get_logger().info("=== 开始关节空间运动规划 ===")
        self.get_logger().info("目标关节位置:")
        for i, (name, pos) in enumerate(zip(self.joint_names, joint_positions)):
            self.get_logger().info(f"  {name}: {pos:.4f} rad ({math.degrees(pos):.2f}°)")
        
        try:
            # 设置目标关节位置
            self.move_group.set_joint_value_target(joint_positions)
            
            # 规划路径
            self.get_logger().info("正在规划路径...")
            plan = self.move_group.plan()
            
            if not plan[0]:
                self.get_logger().error("路径规划失败!")
                return False
            
            self.get_logger().info("路径规划成功!")
            
            if execute:
                # 执行运动
                self.get_logger().info("正在执行运动...")
                success = self.move_group.execute(plan[1], wait=wait)
                if success:
                    self.get_logger().info("运动执行成功!")
                    return True
                else:
                    self.get_logger().error("运动执行失败!")
                    return False
            else:
                self.get_logger().info("仅规划，不执行")
                return True
                
        except Exception as e:
            self.get_logger().error(f"运动过程中发生错误: {e}")
            return False

    def move_to_named_target(self, target_name, execute=True, wait=True):
        """
        移动到预定义的目标位置
        
        Args:
            target_name: 预定义目标名称
            execute: 是否执行运动
            wait: 是否等待运动完成
        
        Returns:
            bool: 运动是否成功
        """
        self.get_logger().info(f"=== 移动到预定义目标: {target_name} ===")
        
        try:
            # 设置命名目标
            self.move_group.set_named_target(target_name)
            
            # 规划并执行
            if execute:
                success = self.move_group.go(wait=wait)
                if success:
                    self.get_logger().info("运动执行成功!")
                    return True
                else:
                    self.get_logger().error("运动执行失败!")
                    return False
            else:
                plan = self.move_group.plan()
                if plan[0]:
                    self.get_logger().info("路径规划成功!")
                    return True
                else:
                    self.get_logger().error("路径规划失败!")
                    return False
                
        except Exception as e:
            self.get_logger().error(f"运动过程中发生错误: {e}")
            return False

    def get_current_pose(self):
        """获取当前末端执行器位姿"""
        return self.move_group.get_current_pose().pose

    def get_current_joint_values(self):
        """获取当前关节位置"""
        return self.move_group.get_current_joint_values()


def main():
    """主函数"""
    rclpy.init()
    
    try:
        # 创建控制器节点
        controller = FrankaEEController()
        
        # 等待用户输入
        controller.get_logger().info("\n=== Franka末端执行器控制器已就绪 ===")
        controller.get_logger().info("按Enter键开始示例运动...")
        input()
        
        # 示例1: 移动到预定义的home位置
        controller.get_logger().info("\n示例1: 移动到home位置")
        controller.move_to_named_target("home")
        time.sleep(2)
        
        # 示例2: 移动到指定的笛卡尔位置
        controller.get_logger().info("\n示例2: 移动到指定笛卡尔位置")
        target_pose = controller.create_pose(
            x=0.4,  # 40cm
            y=0.0,  # 中心
            z=0.6,  # 60cm高度
            roll=0.0,
            pitch=0.0,
            yaw=0.0
        )
        controller.move_to_pose(target_pose)
        time.sleep(2)
        
        # 示例3: 移动到另一个位置
        controller.get_logger().info("\n示例3: 移动到另一个位置")
        target_pose2 = controller.create_pose(
            x=0.5,
            y=0.2,
            z=0.5,
            roll=0.0,
            pitch=0.0,
            yaw=0.0
        )
        controller.move_to_pose(target_pose2)
        time.sleep(2)
        
        # 示例4: 返回home位置
        controller.get_logger().info("\n示例4: 返回home位置")
        controller.move_to_named_target("home")
        
        # 显示最终状态
        controller.get_logger().info("\n=== 运动完成 ===")
        controller.display_current_state()
        
        controller.get_logger().info("\n=== 控制器关闭 ===")
        
    except KeyboardInterrupt:
        controller.get_logger().info("用户中断程序")
    except Exception as e:
        controller.get_logger().error(f"程序运行出错: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
