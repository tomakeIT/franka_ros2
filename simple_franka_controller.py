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
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    PlanningOptions, 
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume
)
from moveit_msgs.srv import GetPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
import math
import time
import tf2_ros
from tf2_ros import TransformException


class SimpleFrankaController(Node):
    """简化的Franka控制器"""
    
    def __init__(self):
        """初始化控制器"""
        super().__init__('simple_franka_controller')
        print("正在初始化Franka控制器...")
        
        # MoveGroup action client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # tf2 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 等待action server
        print("等待MoveGroup action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server不可用!")
            return
        
        # 规划组名称
        self.group_name = 'fr3_arm'  # 正确的FR3规划组名称
        self.end_effector_link = 'fr3_hand_tcp'  # 使用工具中心点(TCP)
        self.planning_frame = 'fr3_link0'  # FR3基础坐标系
        
        # 规划参数
        self.planning_time = 10.0
        self.planning_attempts = 10
        self.velocity_scaling = 0.3
        self.acceleration_scaling = 0.3
        
        # 约束容差
        self.position_tolerance = 0.002  # 目标区域半尺度 (米)
        self.orientation_tolerance = 0.01  # 角度容差 (弧度)
        
        print(f"使用规划组: {self.group_name}")
        print(f"规划框架: {self.planning_frame}")
        print(f"末端执行器链接: {self.end_effector_link}")
        
        # 等待TF准备好
        print("等待TF变换准备...")
        self.wait_for_tf()
        
        print("控制器初始化完成!")

    def wait_for_tf(self):
        """等待TF变换准备好"""
        timeout = rclpy.duration.Duration(seconds=10.0)
        start_time = self.get_clock().now()
        
        while rclpy.ok():
            try:
                if self.tf_buffer.can_transform(
                    self.planning_frame,
                    self.end_effector_link,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=1.0)
                ):
                    print(f"TF变换 {self.planning_frame} -> {self.end_effector_link} 已准备好")
                    return True
                    
                # 检查超时
                if (self.get_clock().now() - start_time) > timeout:
                    self.get_logger().error(f"等待TF变换超时 (10秒)")
                    return False
                    
                # 短暂等待
                rclpy.spin_once(self, timeout_sec=0.1)
                
            except Exception as e:
                self.get_logger().warn(f"等待TF时出错: {e}")
                rclpy.spin_once(self, timeout_sec=0.1)

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
        
        # 创建MoveGroup goal
        goal = MoveGroup.Goal()
        
        # 设置规划请求
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = self.planning_attempts
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.acceleration_scaling
        
        # 创建位置约束
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.planning_frame  # MoveIt规划使用的坐标系
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = target_pose
        
        # 设置目标约束
        constraints = Constraints()
        
        # 位置约束
        position_constraint = PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = self.end_effector_link
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        
        # 创建边界框
        bounding_volume = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [self.position_tolerance * 2] * 3  # 立方体边长
        bounding_volume.primitives.append(box)
        bounding_volume.primitive_poses.append(target_pose)
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        
        constraints.position_constraints.append(position_constraint)
        
        # 方向约束
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose_stamped.header
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
        orientation_constraint.weight = 1.0
        
        constraints.orientation_constraints.append(orientation_constraint)
        
        goal.request.goal_constraints.append(constraints)
        
        # 设置规划选项
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.look_around_attempts = 0
        goal.planning_options.max_safe_execution_cost = 0.0
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        
        print("正在规划路径...")
        
        # 发送goal
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("运动规划被拒绝!")
            return False
        
        print("运动规划已接受，正在执行...")
        
        # 等待结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.result.error_code.val == 1:  # SUCCESS
            print("运动执行成功!")
            return True
        else:
            print(f"运动执行失败! 错误代码: {result.result.error_code.val}")
            return False

    def get_current_position(self):
        """获取当前末端执行器位置"""
        try:
            # 等待TF变换可用
            timeout = rclpy.duration.Duration(seconds=5.0)
            if not self.tf_buffer.can_transform(
                self.planning_frame,
                self.end_effector_link,
                rclpy.time.Time(),
                timeout
            ):
                self.get_logger().warn(f'TF变换 {self.planning_frame} -> {self.end_effector_link} 在5秒内不可用')
                return {
                    'position': [0.0, 0.0, 0.0],
                    'orientation': [0.0, 0.0, 0.0, 1.0]
                }
            
            # 获取从基础坐标系到末端执行器的变换
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame,  # 目标坐标系
                self.end_effector_link,  # 源坐标系
                rclpy.time.Time()  # 最新时间
            )
            
            # 提取位置和方向
            position = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            
            orientation = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            
            return {
                'position': position,
                'orientation': orientation
            }
            
        except TransformException as ex:
            self.get_logger().warn(f'无法获取变换 {self.planning_frame} -> {self.end_effector_link}: {ex}')
            breakpoint()
            return {
                'position': [0.0, 0.0, 0.0],
                'orientation': [0.0, 0.0, 0.0, 1.0]
            }

    def print_current_position(self):
        """打印当前末端执行器位置"""
        pos = self.get_current_position()
        print(f"\n当前末端执行器位置:")
        print(f"  位置: x={pos['position'][0]:.4f}, y={pos['position'][1]:.4f}, z={pos['position'][2]:.4f}")
        # 将四元数转换为欧拉角（roll, pitch, yaw）
        import math
        qx, qy, qz, qw = pos['orientation']
        # 计算roll
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # 计算pitch
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        # 计算yaw
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        print(f"  欧拉角: roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f}")


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
        
        # # 运动1: 移动到指定位置
        # print("\n运动1: 移动到位置 (0.4, 0.0, 0.6) 现在位置:")
        # controller.print_current_position()
        # if controller.move_to_position(0.0, 0.0, 0.0,0,0,0):
        #     controller.print_current_position()
        
        # 运动2: 移动到另一个位置
        print("\n运动2: 移动到位置 (0.1, 0.0, 0.5) 现在位置:")
        # controller.print_current_position()
        if controller.move_to_position(0.5, 0.0, 0.5,-3.14,0,0):
            controller.print_current_position()
        
        # # 运动3: 移动到第三个位置
        # print("\n运动3: 移动到位置 (0.3, -0.2, 0.7) 现在位置:")
        # controller.print_current_position()
        # if controller.move_to_position(0.3, -0.2, 0.7,-3.14,0,0):
        #     controller.print_current_position()
        
        # print("\n=== 运动序列完成 ===")
        
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
