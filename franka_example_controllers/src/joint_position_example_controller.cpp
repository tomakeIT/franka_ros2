// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_example_controllers/joint_position_example_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointPositionExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointPositionExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }

  // add the robot time interface
  if (!is_gazebo_) {
    config.names.push_back(arm_id_ + "/robot_time");
  }

  return config;
}

controller_interface::return_type JointPositionExampleController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
  // On first run, capture current positions as default command
  if (initialization_flag_) {
    std::array<double, 7> current_q{};
    for (int i = 0; i < num_joints; ++i) {
      current_q.at(i) = state_interfaces_[i].get_value();
      initial_q_.at(i) = current_q.at(i);
    }
    command_buffer_.writeFromNonRT(current_q);
    has_command_.store(true);
    // init smoothing state
    current_q_cmd_ = current_q;
    current_q_vel_.fill(0.0);
    initialization_flag_ = false;
    if (!is_gazebo_) {
      initial_robot_time_ = state_interfaces_.back().get_value();
    }
    elapsed_time_ = 0.0;
  } else {
    if (!is_gazebo_) {
      robot_time_ = state_interfaces_.back().get_value();
      elapsed_time_ = robot_time_ - initial_robot_time_;
    } else {
      elapsed_time_ += trajectory_period_;
    }
  }

  // Always read the most recent commanded joint positions as the target
  const std::array<double, 7>* commanded = nullptr;
  if (has_command_.load()) {
    commanded = command_buffer_.readFromRT();
  }
  const std::array<double, 7>& target_q = (commanded != nullptr) ? *commanded : initial_q_;

  // Time step
  const double dt = period.seconds();
  const double vmax = max_joint_velocity_;
  const double amax = max_joint_acceleration_;

  // For each joint, interpolate from current position to target position
  for (int i = 0; i < num_joints; ++i) {
    const double pos_error = target_q.at(i) - current_q_cmd_.at(i);
    
    // If we're very close to target, just set it directly
    if (std::abs(pos_error) < 1e-4) {  // Slightly larger tolerance
      current_q_cmd_.at(i) = target_q.at(i);
      current_q_vel_.at(i) = 0.0;
      command_interfaces_[i].set_value(current_q_cmd_.at(i));
      continue;
    }

    // Calculate desired velocity towards target with smoother transitions
    double v_current = current_q_vel_.at(i);
    const double s = (pos_error >= 0.0) ? 1.0 : -1.0;
    
    // Calculate braking distance needed to stop at target
    const double v_abs = std::abs(v_current);
    const double d_brake = (v_abs * v_abs) / (2.0 * amax) + 1e-4;  // Add small margin

    double v_des = v_current;
    
    // Decide whether to accelerate or decelerate
    if (std::abs(pos_error) <= d_brake) {
      // We need to start braking to reach the target
      const double dv = amax * dt;
      if (v_abs <= dv) {
        // Can stop within this timestep
        v_des = 0.0;
      } else {
        // Decelerate smoothly
        v_des = v_current - std::copysign(dv, v_current);
      }
    } else {
      // We can still accelerate towards target
      const double dv = amax * dt;
      v_des = v_current + s * dv;
      
      // Limit to maximum velocity
      if (std::abs(v_des) > vmax) {
        v_des = std::copysign(vmax, v_des);
      }
    }

    // Update position based on velocity (use average velocity for smoother motion)
    const double v_avg = 0.5 * (v_current + v_des);
    double dq = v_avg * dt;
    
    // Prevent overshoot
    if (std::abs(dq) > std::abs(pos_error)) {
      dq = pos_error;
      v_des = dq / dt;  // Adjust velocity to match actual movement
    }

    current_q_cmd_.at(i) += dq;
    current_q_vel_.at(i) = v_des;
    command_interfaces_[i].set_value(current_q_cmd_.at(i));
    
    // Debug output for joint 0 (can be removed later)
    if (i == 0 && elapsed_time_ > 1.0) {  // Only after initialization
      static int debug_counter = 0;
      if (++debug_counter % 100 == 0) {  // Print every 100 cycles (~0.1s)
        RCLCPP_INFO(get_node()->get_logger(), 
                   "Joint %d: target=%.4f, current=%.4f, error=%.4f, vel=%.4f", 
                   i, target_q.at(i), current_q_cmd_.at(i), pos_error, v_des);
      }
    }
  }

  return controller_interface::return_type::OK;
}

CallbackReturn JointPositionExampleController::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());
  // Subscribe to joint command topic
  auto node = get_node();
  joint_command_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_command", rclcpp::QoS(10),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() < static_cast<size_t>(num_joints)) {
          RCLCPP_WARN(get_node()->get_logger(),
                      "Received JointState with insufficient positions: %zu < %d",
                      msg->position.size(), num_joints);
          return;
        }
        std::array<double, 7> q{};
        for (int i = 0; i < num_joints; ++i) {
          q.at(i) = msg->position[i];
        }
        command_buffer_.writeFromNonRT(q);
        has_command_.store(true);
      });
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerInterface)
