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

#include "ros2_control_demo_example_16/admittance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <kdl_parser/kdl_parser.hpp>

namespace ros2_control_demo_example_16
{

CallbackReturn AdmittanceController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("command_interfaces", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("state_interfaces", std::vector<std::string>());
    auto_declare<std::string>("ft_sensor.name", "");
    auto_declare<std::string>("kinematics.base", "base_link");
    auto_declare<std::string>("kinematics.tip", "tool0");
    auto_declare<std::vector<double>>("admittance.mass", std::vector<double>(6, 5.0));
    auto_declare<std::vector<double>>("admittance.damping", std::vector<double>(6, 50.0));
    auto_declare<std::vector<double>>("admittance.stiffness", std::vector<double>(6, 200.0));
    auto_declare<std::vector<bool>>(
      "admittance.selected_axes", std::vector<bool>(6, true));
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AdmittanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration
AdmittanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  // F/T sensor state interfaces
  conf.names.push_back(ft_sensor_name_ + "/force.x");
  conf.names.push_back(ft_sensor_name_ + "/force.y");
  conf.names.push_back(ft_sensor_name_ + "/force.z");
  conf.names.push_back(ft_sensor_name_ + "/torque.x");
  conf.names.push_back(ft_sensor_name_ + "/torque.y");
  conf.names.push_back(ft_sensor_name_ + "/torque.z");
  return conf;
}

CallbackReturn AdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  command_interface_types_ = get_node()->get_parameter("command_interfaces").as_string_array();
  state_interface_types_ = get_node()->get_parameter("state_interfaces").as_string_array();
  ft_sensor_name_ = get_node()->get_parameter("ft_sensor.name").as_string();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified");
    return CallbackReturn::ERROR;
  }

  if (ft_sensor_name_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No force-torque sensor name specified");
    return CallbackReturn::ERROR;
  }

  num_joints_ = joint_names_.size();

  // Read admittance parameters
  auto mass_vec = get_node()->get_parameter("admittance.mass").as_double_array();
  auto damping_vec = get_node()->get_parameter("admittance.damping").as_double_array();
  auto stiffness_vec = get_node()->get_parameter("admittance.stiffness").as_double_array();
  auto selected_axes_vec = get_node()->get_parameter("admittance.selected_axes").as_bool_array();

  for (int i = 0; i < 6; i++)
  {
    mass_(i) = mass_vec[i];
    damping_(i) = damping_vec[i];
    stiffness_(i) = stiffness_vec[i];
    selected_axes_(i) = selected_axes_vec[i];
  }

  // Initialize admittance state
  cart_velocity_.setZero();
  cart_displacement_.setZero();

  // Initialize reference pose
  ref_position_.setZero();
  ref_orientation_.setIdentity();
  reference_received_ = false;

  // Parse robot description and create KDL chain
  std::string robot_description;
  auto robot_param = rclcpp::Parameter();
  // Get robot description from the controller manager's namespace
  if (get_node()->has_parameter("robot_description"))
  {
    robot_description = get_node()->get_parameter("robot_description").as_string();
  }
  else
  {
    auto parameters_client =
      std::make_shared<rclcpp::SyncParametersClient>(get_node(), "/robot_state_publisher");
    if (parameters_client->wait_for_service(std::chrono::seconds(5)))
    {
      auto parameters = parameters_client->get_parameters({"robot_description"});
      if (!parameters.empty())
      {
        robot_description = parameters[0].as_string();
      }
    }
  }

  if (robot_description.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description");
    return CallbackReturn::ERROR;
  }

  KDL::Tree robot_tree;
  if (!kdl_parser::treeFromString(robot_description, robot_tree))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse robot description into KDL tree");
    return CallbackReturn::ERROR;
  }

  std::string base = get_node()->get_parameter("kinematics.base").as_string();
  std::string tip = get_node()->get_parameter("kinematics.tip").as_string();
  if (!robot_tree.getChain(base, tip, kdl_chain_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Failed to get KDL chain from '%s' to '%s'", base.c_str(),
      tip.c_str());
    return CallbackReturn::ERROR;
  }

  // Create KDL solvers
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);

  RCLCPP_INFO(
    get_node()->get_logger(), "KDL chain: %s -> %s with %d joints", base.c_str(), tip.c_str(),
    kdl_chain_.getNrOfJoints());

  // Create target pose subscriber
  target_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/target_pose", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { rt_target_pose_.writeFromNonRT(*msg); });

  // Create wrench publisher for visualization
  wrench_pub_ =
    get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>("~/measured_wrench", 10);

  RCLCPP_INFO(get_node()->get_logger(), "Admittance controller configured successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AdmittanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Sort and assign command interfaces
  joint_position_command_interface_.clear();
  joint_velocity_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();
  ft_state_interface_.clear();

  for (auto & interface : command_interfaces_)
  {
    for (size_t j = 0; j < joint_names_.size(); j++)
    {
      if (interface.get_prefix_name() == joint_names_[j])
      {
        if (interface.get_interface_name() == "position")
        {
          joint_position_command_interface_.emplace_back(interface);
        }
        else if (interface.get_interface_name() == "velocity")
        {
          joint_velocity_command_interface_.emplace_back(interface);
        }
      }
    }
  }

  for (auto & interface : state_interfaces_)
  {
    for (size_t j = 0; j < joint_names_.size(); j++)
    {
      if (interface.get_prefix_name() == joint_names_[j])
      {
        if (interface.get_interface_name() == "position")
        {
          joint_position_state_interface_.emplace_back(interface);
        }
        else if (interface.get_interface_name() == "velocity")
        {
          joint_velocity_state_interface_.emplace_back(interface);
        }
      }
    }
    if (interface.get_prefix_name() == ft_sensor_name_)
    {
      ft_state_interface_.emplace_back(interface);
    }
  }

  // Reset admittance state
  cart_velocity_.setZero();
  cart_displacement_.setZero();
  reference_received_ = false;

  // Set initial reference from current FK
  KDL::JntArray q(num_joints_);
  for (unsigned int i = 0; i < num_joints_; i++)
  {
    q(i) = joint_position_state_interface_[i].get().get_value();
  }
  computeFK(q, ref_position_, ref_orientation_);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Admittance controller activated. Initial TCP position: [%.3f, %.3f, %.3f]", ref_position_(0),
    ref_position_(1), ref_position_(2));

  return CallbackReturn::SUCCESS;
}

CallbackReturn AdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type AdmittanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0)
  {
    return controller_interface::return_type::OK;
  }

  // Read current joint positions
  KDL::JntArray q(num_joints_);
  KDL::JntArray q_dot(num_joints_);
  for (unsigned int i = 0; i < num_joints_; i++)
  {
    q(i) = joint_position_state_interface_[i].get().get_value();
    q_dot(i) = joint_velocity_state_interface_[i].get().get_value();
  }

  // Get current TCP pose via FK
  Eigen::Vector3d current_position;
  Eigen::Matrix3d current_orientation;
  computeFK(q, current_position, current_orientation);

  // Get Jacobian at current configuration
  KDL::Jacobian jac(num_joints_);
  jac_solver_->JntToJac(q, jac);

  // Read F/T sensor data
  Eigen::Matrix<double, 6, 1> ft_wrench;
  ft_wrench.setZero();
  if (ft_state_interface_.size() == 6)
  {
    for (int i = 0; i < 6; i++)
    {
      ft_wrench(i) = ft_state_interface_[i].get().get_value();
    }
  }

  // Publish measured wrench for visualization
  geometry_msgs::msg::WrenchStamped wrench_msg;
  wrench_msg.header.stamp = time;
  wrench_msg.header.frame_id = "tool0";
  wrench_msg.wrench.force.x = ft_wrench(0);
  wrench_msg.wrench.force.y = ft_wrench(1);
  wrench_msg.wrench.force.z = ft_wrench(2);
  wrench_msg.wrench.torque.x = ft_wrench(3);
  wrench_msg.wrench.torque.y = ft_wrench(4);
  wrench_msg.wrench.torque.z = ft_wrench(5);
  wrench_pub_->publish(wrench_msg);

  // Update reference from subscriber if available
  auto target_pose = *rt_target_pose_.readFromRT();
  if (target_pose.header.stamp.sec != 0 || target_pose.header.stamp.nanosec != 0)
  {
    ref_position_(0) = target_pose.pose.position.x;
    ref_position_(1) = target_pose.pose.position.y;
    ref_position_(2) = target_pose.pose.position.z;

    // Convert quaternion to rotation matrix
    double qx = target_pose.pose.orientation.x;
    double qy = target_pose.pose.orientation.y;
    double qz = target_pose.pose.orientation.z;
    double qw = target_pose.pose.orientation.w;
    Eigen::Quaterniond quat(qw, qx, qy, qz);
    quat.normalize();
    ref_orientation_ = quat.toRotationMatrix();
    reference_received_ = true;
  }

  // Compute pose error (Cartesian)
  Eigen::Matrix<double, 6, 1> pose_error;
  pose_error.head<3>() = current_position - ref_position_;
  pose_error.tail<3>() = computeOrientationError(ref_orientation_, current_orientation);

  // Admittance control law:
  //   M * a + D * v + K * x = F_ext
  //   a = M^{-1} * (F_ext - D * v - K * x)
  //   v_new = v + a * dt
  //   x_new = x + v_new * dt
  //
  // Then: desired_position = reference + x_new (displacement from reference)
  Eigen::Matrix<double, 6, 1> cart_accel;
  for (int i = 0; i < 6; i++)
  {
    if (selected_axes_(i))
    {
      cart_accel(i) =
        (ft_wrench(i) - damping_(i) * cart_velocity_(i) - stiffness_(i) * cart_displacement_(i)) /
        mass_(i);
    }
    else
    {
      cart_accel(i) = 0.0;
      cart_velocity_(i) = 0.0;
      cart_displacement_(i) = 0.0;
    }
  }

  // Integrate velocity and displacement
  cart_velocity_ += cart_accel * dt;
  cart_displacement_ += cart_velocity_ * dt;

  // Compute desired Cartesian pose = reference + displacement
  Eigen::Matrix<double, 6, 1> desired_cart_vel = cart_velocity_;

  // Desired position = reference + displacement
  Eigen::Vector3d desired_position = ref_position_ + cart_displacement_.head<3>();

  // Compute position error for joint-level control
  Eigen::Matrix<double, 6, 1> cart_error;
  cart_error.head<3>() = desired_position - current_position;
  cart_error.tail<3>() = -cart_displacement_.tail<3>();  // orientation error correction

  // Use Jacobian pseudo-inverse to convert Cartesian to joint-space
  Eigen::MatrixXd J = jac.data;
  Eigen::MatrixXd J_pinv;

  // Damped pseudo-inverse for numerical stability
  double lambda = 0.01;
  Eigen::MatrixXd JJt = J * J.transpose();
  JJt.diagonal() += Eigen::VectorXd::Constant(6, lambda * lambda);
  J_pinv = J.transpose() * JJt.inverse();

  // Compute joint velocity command: q_dot = J^+ * (K_p * error + desired_vel)
  double kp = 5.0;  // proportional gain for tracking
  Eigen::Matrix<double, 6, 1> cart_cmd = kp * cart_error + desired_cart_vel;
  Eigen::VectorXd joint_vel_cmd = J_pinv * cart_cmd;

  // Integrate to get joint position commands
  for (unsigned int i = 0; i < num_joints_; i++)
  {
    double new_pos = q(i) + joint_vel_cmd(i) * dt;
    joint_position_command_interface_[i].get().set_value(new_pos);
    if (!joint_velocity_command_interface_.empty())
    {
      joint_velocity_command_interface_[i].get().set_value(joint_vel_cmd(i));
    }
  }

  return controller_interface::return_type::OK;
}

void AdmittanceController::computeFK(
  const KDL::JntArray & joint_positions, Eigen::Vector3d & position,
  Eigen::Matrix3d & orientation)
{
  KDL::Frame frame;
  fk_solver_->JntToCart(joint_positions, frame);

  position(0) = frame.p.x();
  position(1) = frame.p.y();
  position(2) = frame.p.z();

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      orientation(i, j) = frame.M(i, j);
    }
  }
}

Eigen::Vector3d AdmittanceController::computeOrientationError(
  const Eigen::Matrix3d & R_desired, const Eigen::Matrix3d & R_current)
{
  Eigen::Matrix3d R_error = R_desired.transpose() * R_current;
  // Convert rotation matrix error to angle-axis
  Eigen::AngleAxisd angle_axis(R_error);
  return R_current * (angle_axis.angle() * angle_axis.axis());
}

}  // namespace ros2_control_demo_example_16

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_16::AdmittanceController,
  controller_interface::ControllerInterface)
