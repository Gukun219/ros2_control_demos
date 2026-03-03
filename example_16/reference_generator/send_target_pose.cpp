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

  // Send a circular trajectory of target poses
  double total_time = 10.0;
  double dt = 0.01;
  int num_points = static_cast<int>(total_time / dt);

  // Home position of the tool0 frame (approximate for r6bot at zero configuration)
  double home_x = 0.0;
  double home_y = 0.0;
  double home_z = 1.0;

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

    // Circular motion in the XY plane
    target_pose.pose.position.x = home_x + radius * std::cos(angle);
    target_pose.pose.position.y = home_y + radius * std::sin(angle);
    target_pose.pose.position.z = home_z;

    // Keep orientation fixed (identity quaternion)
    target_pose.pose.orientation.x = 0.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 1.0;

    pub->publish(target_pose);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Target pose sequence complete.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
