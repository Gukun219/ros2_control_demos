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

#include <cmath>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("send_target_pose");
  auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
    "/admittance_controller/joint_references", 10);

  // Get robot description and build KDL chain
  node->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
  auto robot_param = rclcpp::Parameter();
  node->get_parameter("robot_description", robot_param);
  auto robot_description = robot_param.as_string();

  KDL::Tree robot_tree;
  KDL::Chain chain;
  kdl_parser::treeFromString(robot_description, robot_tree);
  robot_tree.getChain("base_link", "tool0", chain);

  unsigned int n_joints = chain.getNrOfJoints();
  auto joint_positions = KDL::JntArray(n_joints);
  auto joint_velocities = KDL::JntArray(n_joints);
  auto twist = KDL::Twist();

  // Create KDL solvers
  auto fk_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
  auto ik_vel_solver = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain, 0.0000001);

  // Compute FK at home (all-zero) to get circle center
  KDL::Frame home_frame;
  fk_solver->JntToCart(joint_positions, home_frame);
  double home_x = home_frame.p.x();
  double home_y = home_frame.p.y();
  double home_z = home_frame.p.z();

  RCLCPP_INFO(
    node->get_logger(), "Home position from FK: (%.3f, %.3f, %.3f)", home_x, home_y, home_z);

  // Circular trajectory parameters
  double radius = 0.05;
  double period = 10.0;
  double omega = 2.0 * M_PI / period;
  double dt = 0.01;  // 100 Hz, matching controller update rate

  RCLCPP_INFO(
    node->get_logger(),
    "Publishing joint references for circular trajectory (radius=%.3f m, period=%.1f s)...",
    radius, period);

  // Wait for controllers to start
  RCLCPP_INFO(node->get_logger(), "Waiting 3 seconds for controllers to start...");
  rclcpp::sleep_for(3s);

  trajectory_msgs::msg::JointTrajectoryPoint point_msg;
  point_msg.positions.resize(n_joints);
  point_msg.velocities.resize(n_joints);

  rclcpp::Rate rate(1.0 / dt);
  double angle = 0.0;

  while (rclcpp::ok())
  {
    // Cartesian velocity: tangent to the circle
    twist.vel.x(-radius * omega * std::sin(angle));
    twist.vel.y(radius * omega * std::cos(angle));
    twist.vel.z(0.0);
    twist.rot.x(0.0);
    twist.rot.y(0.0);
    twist.rot.z(0.0);

    // Convert Cartesian twist to joint velocities via Jacobian pseudo-inverse
    ik_vel_solver->CartToJnt(joint_positions, twist, joint_velocities);

    // Publish current joint state as reference
    std::memcpy(
      point_msg.positions.data(), joint_positions.data.data(),
      n_joints * sizeof(double));
    std::memcpy(
      point_msg.velocities.data(), joint_velocities.data.data(),
      n_joints * sizeof(double));

    pub->publish(point_msg);

    // Integrate joint velocities to update positions
    joint_positions.data += joint_velocities.data * dt;

    // Advance angle
    angle += omega * dt;

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
