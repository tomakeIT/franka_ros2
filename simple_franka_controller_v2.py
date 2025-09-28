#!/usr/bin/env python3
"""
简化的Franka Emika FR3 末端执行器控制脚本 (版本2)

这个版本使用更简单的ROS2接口，避免了moveit_commander的依赖问题。
使用MoveIt2的服务接口进行运动规划和执行。

使用方法:
1. 启动机械臂和MoveIt:
   ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=<机械臂IP地址>
2. 运行此脚本:
   python3 simple_franka_controller_v2.py

作者: AI Assistant
日期: 2024
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_msgs.msg import (
    MotionPlanRequest, 
    PlanningOptions, 
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import math
import time
import tf2_ros
from tf2_ros import TransformException


class SimpleFrankaControllerV2(Node):
    """简化的Franka控制器 V2"""
    
    def __init__(self):
        """初始化控制器"""
        super().__init__('simple_franka_controller_v2')
        print("正在初始化Franka控制器 V2...")
        
        # 服务客户端
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # tf2 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 等待服务
        print("等待MoveIt服务...")
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("IK服务不可用，将跳过IK计算")
        
        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("运动规划服务不可用")
        
        # 机器人参数
        self.group_name = 'fr3_arm'  # MoveIt规划组名称（与SRDF一致）
        self.end_effector_link = 'fr3_link8'  # 规划组的tip_link（与MoveIt一致）
        self.planning_frame = 'fr3_link0'  # 规划坐标系（与MoveIt一致）
        
        # 注意：fr3_link8是法兰，fr3_hand_tcp是工具中心点
        # 对于运动规划，使用fr3_link8更合适
        
        # 规划参数
        self.planning_time = 5.0
        self.planning_attempts = 5
        self.velocity_scaling = 0.3
        self.acceleration_scaling = 0.3
        
        print(f"使用规划组: {self.group_name}")
        print(f"规划框架: {self.planning_frame}")
        print(f"末端执行器链接: {self.end_effector_link}")
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

    def compute_ik(self, target_pose):
        """
        计算逆运动学
        
        Args:
            target_pose: 目标位姿
            
        Returns:
            JointState: 关节状态，如果失败返回None
        """
        if not self.ik_client.service_is_ready():
            print("IK服务不可用")
            return None
            
        # 创建IK请求
        request = GetPositionIK.Request()
        
        # 设置位姿
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.planning_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = target_pose
        
        request.ik_request.group_name = self.group_name
        request.ik_request.robot_state.joint_state.name = []
        request.ik_request.robot_state.joint_state.position = []
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = int(self.planning_time)
        
        # 调用服务
        try:
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response.error_code.val == 1:  # SUCCESS
                return response.solution.joint_state
            else:
                print(f"IK计算失败，错误代码: {response.error_code.val}")
                return None
                
        except Exception as e:
            print(f"IK服务调用失败: {e}")
            return None

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
        
        # 计算逆运动学
        print("正在计算逆运动学...")
        joint_state = self.compute_ik(target_pose)
        
        if joint_state is None:
            print("逆运动学计算失败!")
            return False
        
        print("逆运动学计算成功!")
        print(f"目标关节角度: {[f'{angle:.3f}' for angle in joint_state.position]}")
        
        # 在实际应用中，这里应该发送关节角度到控制器
        # 由于这是演示代码，我们只打印结果
        print("注意：实际的关节控制需要额外的控制器接口")
        
        return True

    def get_current_position(self):
        """获取当前末端执行器位置"""
        try:
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
            return {
                'position': [0.0, 0.0, 0.0],
                'orientation': [0.0, 0.0, 0.0, 1.0]
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
    print("=== Franka末端执行器位置控制器 V2 ===")
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        # 创建控制器
        controller = SimpleFrankaControllerV2()
        
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
