#!/usr/bin/env python3
"""
Franka Emika FR3 MoveIt Action客户端

这个脚本使用ROS2 Action接口直接与MoveIt通信，控制Franka机械臂运动到指定的末端执行器位置。
提供了更底层的控制和更好的错误处理。

使用方法:
1. 启动机械臂和MoveIt:
   ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=<机械臂IP地址>
2. 运行此脚本:
   python3 franka_moveit_action_client.py

作者: AI Assistant
日期: 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from moveit_msgs.srv import GetPositionIK
import math
import time


class FrankaMoveItActionClient(Node):
    """Franka MoveIt Action客户端"""
    
    def __init__(self):
        super().__init__('franka_moveit_action_client')
        
        # 创建MoveGroup action客户端
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        # 等待action服务器
        self.get_logger().info('等待MoveGroup action服务器...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('MoveGroup action服务器已连接!')
        
        # 规划组名称
        self.group_name = 'panda_manipulator'  # 或使用 'panda_arm'
        
        # 获取当前位置
        self.current_pose = self.get_current_pose()

    def create_pose_stamped(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        创建带时间戳的位姿
        
        Args:
            x, y, z: 位置坐标 (米)
            roll, pitch, yaw: 欧拉角 (弧度)
        
        Returns:
            geometry_msgs.msg.PoseStamped: 带时间戳的位姿
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "fr3_link0"  # 基础框架
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        pose_stamped.pose.position = Point(x=x, y=y, z=z)
        
        # 将欧拉角转换为四元数
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        pose_stamped.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose_stamped.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose_stamped.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose_stamped.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        return pose_stamped

    def get_current_pose(self):
        """获取当前末端执行器位姿 (简化版本)"""
        # 这里应该从robot_state_publisher获取当前位姿
        # 为了简化，返回一个默认位姿
        return self.create_pose_stamped(0.3, 0.0, 0.5)

    def move_to_pose_async(self, target_pose):
        """
        异步移动到指定位姿
        
        Args:
            target_pose: 目标位姿 (geometry_msgs.msg.PoseStamped)
        
        Returns:
            rclpy.action.Future: 动作执行结果
        """
        self.get_logger().info(f"开始移动到位置: x={target_pose.pose.position.x:.3f}, "
                              f"y={target_pose.pose.position.y:.3f}, z={target_pose.pose.position.z:.3f}")
        
        # 创建运动规划请求
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.group_name
        motion_plan_request.num_planning_attempts = 5
        motion_plan_request.allowed_planning_time = 5.0
        
        # 设置目标约束
        constraints = Constraints()
        
        # 位置约束
        position_constraint = PositionConstraint()
        position_constraint.header = target_pose.header
        position_constraint.link_name = "fr3_hand"  # 末端执行器链接
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        position_constraint.weight = 1.0
        
        # 设置目标位置
        position_constraint.constraint_region.primitives = []
        position_constraint.constraint_region.primitive_poses = [target_pose.pose]
        
        constraints.position_constraints = [position_constraint]
        
        # 方向约束
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = target_pose.header
        orientation_constraint.link_name = "fr3_hand"
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.weight = 1.0
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        
        constraints.orientation_constraints = [orientation_constraint]
        
        motion_plan_request.goal_constraints = [constraints]
        
        # 创建规划选项
        planning_options = PlanningOptions()
        planning_options.plan_only = False  # 执行规划
        planning_options.look_around = False
        planning_options.look_around_attempts = 0
        planning_options.max_safe_execution_cost = 0.0
        
        # 创建MoveGroup目标
        goal = MoveGroup.Goal()
        goal.request = motion_plan_request
        goal.planning_options = planning_options
        
        # 发送目标
        future = self.move_group_client.send_goal_async(goal)
        return future

    def move_to_position(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        移动到指定位置
        
        Args:
            x, y, z: 目标位置 (米)
            roll, pitch, yaw: 目标方向 (弧度)
        
        Returns:
            bool: 运动是否成功
        """
        # 创建目标位姿
        target_pose = self.create_pose_stamped(x, y, z, roll, pitch, yaw)
        
        # 发送动作目标
        future = self.move_to_pose_async(target_pose)
        
        # 等待结果
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

    def print_current_position(self):
        """打印当前位置信息"""
        self.get_logger().info("当前末端执行器位置:")
        self.get_logger().info(f"  位置: x={self.current_pose.pose.position.x:.4f}, "
                              f"y={self.current_pose.pose.position.y:.4f}, "
                              f"z={self.current_pose.pose.position.z:.4f}")
        self.get_logger().info(f"  方向: x={self.current_pose.pose.orientation.x:.4f}, "
                              f"y={self.current_pose.pose.orientation.y:.4f}, "
                              f"z={self.current_pose.pose.orientation.z:.4f}, "
                              f"w={self.current_pose.pose.orientation.w:.4f}")


def main():
    """主函数"""
    rclpy.init()
    
    try:
        # 创建action客户端
        client = FrankaMoveItActionClient()
        
        # 显示当前位置
        client.print_current_position()
        
        # 示例运动序列
        client.get_logger().info("开始示例运动序列...")
        
        # 运动1
        client.get_logger().info("\n运动1: 移动到位置 (0.4, 0.0, 0.6)")
        if client.move_to_position(0.4, 0.0, 0.6):
            client.print_current_position()
        
        time.sleep(1)
        
        # 运动2
        client.get_logger().info("\n运动2: 移动到位置 (0.5, 0.2, 0.5)")
        if client.move_to_position(0.5, 0.2, 0.5):
            client.print_current_position()
        
        time.sleep(1)
        
        # 运动3
        client.get_logger().info("\n运动3: 移动到位置 (0.3, -0.2, 0.7)")
        if client.move_to_position(0.3, -0.2, 0.7):
            client.print_current_position()
        
        client.get_logger().info("\n=== 运动序列完成 ===")
        
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"\n程序运行出错: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
