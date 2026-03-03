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

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_16__ADMITTANCE_CONTROLLER_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_16__ADMITTANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace ros2_control_demo_example_16
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AdmittanceController : public controller_interface::ControllerInterface
{
public:
  AdmittanceController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // Joint names and interfaces
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  // Sensor configuration
  std::string ft_sensor_name_;

  // KDL kinematic chain
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  unsigned int num_joints_;

  // Admittance parameters (6-DOF: x, y, z, rx, ry, rz)
  Eigen::Matrix<double, 6, 1> mass_;
  Eigen::Matrix<double, 6, 1> damping_;
  Eigen::Matrix<double, 6, 1> stiffness_;
  Eigen::Matrix<bool, 6, 1> selected_axes_;

  // Admittance state variables (in Cartesian space)
  Eigen::Matrix<double, 6, 1> cart_velocity_;  // current Cartesian velocity
  Eigen::Matrix<double, 6, 1> cart_displacement_;  // displacement from reference

  // Reference pose (from subscriber)
  Eigen::Vector3d ref_position_;
  Eigen::Matrix3d ref_orientation_;
  bool reference_received_;

  // Target pose subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped> rt_target_pose_;

  // Wrench publisher (for visualization)
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;

  // Command/state interface references
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_position_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_velocity_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_velocity_state_interface_;

  // F/T sensor state interface indices
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    ft_state_interface_;

  // Helper to compute forward kinematics
  void computeFK(
    const KDL::JntArray & joint_positions, Eigen::Vector3d & position,
    Eigen::Matrix3d & orientation);

  // Helper to compute orientation error
  Eigen::Vector3d computeOrientationError(
    const Eigen::Matrix3d & R_desired, const Eigen::Matrix3d & R_current);
};

}  // namespace ros2_control_demo_example_16

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_16__ADMITTANCE_CONTROLLER_HPP_
