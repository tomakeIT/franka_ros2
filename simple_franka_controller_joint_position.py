#!/usr/bin/env python3
"""
Simplified Franka Emika FR3 Joint Position Control Script

This is a simplified version specifically for controlling Franka robot arm 
to move to specified joint positions. Uses MoveIt2 Python interface for 
joint space motion control.

Usage:
1. Launch robot arm and MoveIt:
   ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=<robot_ip_address>
2. Run this script:
   python3 simple_franka_controller_joint_position.py

Author: AI Assistant
Date: 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# No longer need geometry messages since using joint position control
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    PlanningOptions, 
    Constraints,
    JointConstraint
)
from sensor_msgs.msg import JointState
import threading
from collections import deque
import math
import time


class SimpleFrankaController(Node):
    """Simplified Franka Controller"""
    
    def __init__(self):
        """Initialize the controller"""
        super().__init__('simple_franka_controller')
        print("Initializing Franka controller...")
        
        # MoveGroup action client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Wait for action server
        print("Waiting for MoveGroup action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            return
        
        # Planning group name
        self.group_name = 'fr3_arm'  # Correct FR3 planning group name
        self.planning_frame = 'fr3_link0'  # FR3 base coordinate frame
        
        # Franka FR3 joint names
        self.joint_names = [
            'fr3_joint1',
            'fr3_joint2', 
            'fr3_joint3',
            'fr3_joint4',
            'fr3_joint5',
            'fr3_joint6',
            'fr3_joint7'
        ]
        
        # Planning parameters
        self.planning_time = 10.0
        self.planning_attempts = 10
        self.velocity_scaling = 0.3
        self.acceleration_scaling = 0.3
        
        # Joint constraint tolerance
        self.joint_tolerance = 0.01  # Joint angle tolerance (radians)
        
        print(f"Using planning group: {self.group_name}")
        print(f"Planning frame: {self.planning_frame}")
        print(f"Joint names: {self.joint_names}")
        
        # Command subscription (desired joint targets)
        self._cmd_queue = deque()
        self._is_executing = False
        self._joint_command_sub = self.create_subscription(
            JointState,
            '/joint_commands',
            self._joint_command_callback,
            10
        )
        # Periodic processor to launch execution in background
        self._cmd_timer = self.create_timer(0.05, self._process_joint_commands)

        print("Controller initialization complete!")

    def validate_joint_positions(self, joint_positions):
        """
        Validate if joint positions are valid
        
        Args:
            joint_positions: List of joint angles (radians)
        
        Returns:
            bool: Whether joint positions are valid
        """
        if len(joint_positions) != len(self.joint_names):
            self.get_logger().error(f"Joint position count mismatch! Expected {len(self.joint_names)}, got {len(joint_positions)}")
            return False
        
        # Check if joint angles are within reasonable range
        for i, angle in enumerate(joint_positions):
            if not isinstance(angle, (int, float)):
                self.get_logger().error(f"Joint {i+1} angle is not a number: {angle}")
                return False
                
        return True

    def move_to_joint_positions(self, joint_positions):
        """
        Move to specified joint positions
        
        Args:
            joint_positions: List of joint angles (radians), corresponding to fr3_joint1 to fr3_joint7 in order
        
        Returns:
            bool: Whether motion was successful
        """
        print(f"\n=== Moving to joint positions: {[f'{angle:.3f}' for angle in joint_positions]} ===")
        # return True
        
        # Validate joint positions
        if not self.validate_joint_positions(joint_positions):
            return False
        
        # Create MoveGroup goal
        goal = MoveGroup.Goal()
        
        # Set planning request
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = self.planning_attempts
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.acceleration_scaling
        
        # Set goal constraints
        constraints = Constraints()
        
        # Create constraints for each joint
        for i, (joint_name, target_angle) in enumerate(zip(self.joint_names, joint_positions)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = target_angle
            joint_constraint.tolerance_above = self.joint_tolerance
            joint_constraint.tolerance_below = self.joint_tolerance
            joint_constraint.weight = 1.0
            
            constraints.joint_constraints.append(joint_constraint)
        
        goal.request.goal_constraints.append(constraints)
        
        # Set planning options
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.look_around_attempts = 0
        goal.planning_options.max_safe_execution_cost = 0.0
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 3
        
        print("Planning path...")
        
        # Send goal asynchronously and handle via callbacks
        send_future = self.move_group_client.send_goal_async(goal)

        def _on_result(fut):
            try:
                result = fut.result()
                code_val = 'unknown'
                ok = False
                try:
                    code_val = result.result.error_code.val
                    ok = (code_val == 1)
                except Exception:
                    pass
                if ok:
                    print("Motion execution successful!")
                else:
                    print(f"Motion execution failed! Error code: {code_val}")
            except Exception as exc:
                print(f"Motion execution error: {exc}")
            finally:
                self._is_executing = False

        def _on_goal_response(fut):
            try:
                goal_handle = fut.result()
            except Exception as exc:
                print(f"Motion planning error: {exc}")
                self._is_executing = False
                return

            if not goal_handle or not goal_handle.accepted:
                print("Motion planning rejected!")
                self._is_executing = False
                return

            print("Motion planning accepted, executing...")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_on_result)

        send_future.add_done_callback(_on_goal_response)
        # Return immediately; result handled asynchronously
        return True

    def _joint_command_callback(self, msg: JointState):
        """Callback for desired joint commands on topic 'joint_commands'."""
        try:
            target_positions = []
            if msg.name and len(msg.name) == len(msg.position):
                name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
                for joint_name in self.joint_names:
                    if joint_name in name_to_pos:
                        target_positions.append(name_to_pos[joint_name])
                    else:
                        self.get_logger().error(f"Command missing joint {joint_name}")
                        return
            else:
                if len(msg.position) != len(self.joint_names):
                    self.get_logger().error(
                        f"Command length mismatch: expected {len(self.joint_names)}, got {len(msg.position)}"
                    )
                    return
                target_positions = list(msg.position)

            # Enqueue command
            self._cmd_queue.append(target_positions)
        except Exception as e:
            self.get_logger().error(f"Error in joint command callback: {e}")

    def _process_joint_commands(self):
        """Periodically check queue and execute next command in a background thread."""
        if self._is_executing:
            return
        if not self._cmd_queue:
            return

        targets = self._cmd_queue.popleft()
        self._is_executing = True

        try:
            self.move_to_joint_positions(targets)
        except Exception as e:
            self.get_logger().error(f"Failed to start joint motion: {e}")
            self._is_executing = False


def main():
    """Main function"""
    print("=== Franka Joint Position Controller ===")
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create controller
        controller = SimpleFrankaController()
        
        print("Waiting for 'joint_commands' (sensor_msgs/JointState). Press Ctrl+C to quit.")
        # Keep spinning to process incoming commands
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("\nUser interrupted program")
    except Exception as e:
        print(f"\nProgram error: {e}")
    finally:
        # Destroy node
        controller.destroy_node()
        # Shutdown ROS 2
        rclpy.shutdown()


if __name__ == '__main__':
    main()
