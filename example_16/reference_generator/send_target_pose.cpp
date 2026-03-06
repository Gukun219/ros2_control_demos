// Copyright 2023 ros2_control Development Team
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

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("send_target_pose");
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/admittance_controller/target_pose", 10);

  // Wait for the admittance controller to be active
  RCLCPP_INFO(node->get_logger(), "Waiting 3 seconds for controllers to start...");
  rclcpp::sleep_for(3s);

  // Look up the actual tool0 pose in base_link frame as the home position.
  // This avoids hardcoding a value that may not match the robot's real FK at zero config.
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  RCLCPP_INFO(node->get_logger(), "Looking up tool0 pose in base_link frame...");
  geometry_msgs::msg::TransformStamped tool0_transform;
  while (rclcpp::ok()) {
    try {
      tool0_transform = tf_buffer->lookupTransform("base_link", "tool0", tf2::TimePointZero);
      break;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Waiting for TF: %s", ex.what());
      rclcpp::sleep_for(100ms);
    }
  }

  // Use the actual FK position of tool0 as the circle center
  double home_x = tool0_transform.transform.translation.x;
  double home_y = tool0_transform.transform.translation.y;
  double home_z = tool0_transform.transform.translation.z;
  // Use the actual FK orientation of tool0 to avoid IK orientation mismatch
  double home_qx = tool0_transform.transform.rotation.x;
  double home_qy = tool0_transform.transform.rotation.y;
  double home_qz = tool0_transform.transform.rotation.z;
  double home_qw = tool0_transform.transform.rotation.w;

  RCLCPP_INFO(
    node->get_logger(), "Home position: (%.3f, %.3f, %.3f)", home_x, home_y, home_z);

  // Send a circular trajectory of target poses
  double total_time = 10.0;
  double dt = 0.01;
  int num_points = static_cast<int>(total_time / dt);

  // Circular motion radius
  double radius = 0.1;

  RCLCPP_INFO(
    node->get_logger(), "Publishing target poses for circular motion (radius=%.2f m, T=%.1f s)...",
    radius, total_time);

  rclcpp::Rate rate(1.0 / dt);
  for (int i = 0; i < num_points && rclcpp::ok(); i++)
  {
    double t = static_cast<double>(i) / num_points;
    double angle = 2.0 * M_PI * t;

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.stamp = node->now();
    target_pose.header.frame_id = "base_link";

    // Circular motion in the XY plane around the actual home position
    target_pose.pose.position.x = home_x + radius * std::cos(angle);
    target_pose.pose.position.y = home_y + radius * std::sin(angle);
    target_pose.pose.position.z = home_z;

    // Use the actual tool0 orientation to avoid IK singularities
    target_pose.pose.orientation.x = home_qx;
    target_pose.pose.orientation.y = home_qy;
    target_pose.pose.orientation.z = home_qz;
    target_pose.pose.orientation.w = home_qw;

    pub->publish(target_pose);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Target pose sequence complete.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
