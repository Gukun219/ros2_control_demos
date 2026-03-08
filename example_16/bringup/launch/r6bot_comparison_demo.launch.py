# Copyright 2024 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Admittance Controller Comparison Demo Launch

Launches robot + admittance controller + data recorder.
No trajectory commands — robot stays at home position.
Manually switch stiffness in another terminal to compare.

Usage:
  # Terminal 1: launch demo
  ros2 launch ros2_control_demo_example_16 r6bot_comparison_demo.launch.py

  # Terminal 2: switch stiffness manually
  ros2 param set /admittance_controller admittance.stiffness \
      "[10000.0, 10000.0, 10000.0, 1000.0, 1000.0, 1000.0]"
  ros2 param set /admittance_controller admittance.stiffness \
      "[200.0, 200.0, 200.0, 20.0, 20.0, 20.0]"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Start RViz2 automatically.",
        ),
        DeclareLaunchArgument(
            "output_dir", default_value="/tmp/admittance_comparison",
            description="Directory to save recorded data and plots.",
        ),
    ]

    gui = LaunchConfiguration("gui")
    output_dir = LaunchConfiguration("output_dir")

    # URDF
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ros2_control_demo_example_16"),
            "urdf", "r6bot.urdf.xacro",
        ]),
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("ros2_control_demo_example_16"),
        "config", "r6bot_admittance_controller.yaml",
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ros2_control_demo_description"),
        "r6bot/rviz", "view_robot.rviz",
    ])

    # --- Core nodes ---
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[("~/robot_description", "/robot_description")],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        output="log", arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # --- Controller spawners (sequential) ---
    jsb_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager",
                    "/controller_manager"],
    )
    fts_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["force_torque_sensor_broadcaster", "--controller-manager",
                    "/controller_manager"],
    )
    admittance_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["admittance_controller", "-c", "/controller_manager"],
    )

    # --- Data recorder (continuous, Ctrl+C to stop and plot) ---
    recorder_node = Node(
        package="ros2_control_demo_example_16",
        executable="record_and_compare",
        name="data_recorder",
        parameters=[{"output_dir": output_dir}],
        output="screen",
    )

    # --- Event chain for sequential startup ---
    delay_rviz = RegisterEventHandler(OnProcessExit(
        target_action=jsb_spawner,
        on_exit=[rviz_node],
    ))
    delay_fts = RegisterEventHandler(OnProcessExit(
        target_action=jsb_spawner,
        on_exit=[fts_spawner],
    ))
    delay_admittance = RegisterEventHandler(OnProcessExit(
        target_action=fts_spawner,
        on_exit=[admittance_spawner],
    ))
    # Start recorder after admittance controller is up
    delay_recorder = RegisterEventHandler(OnProcessExit(
        target_action=admittance_spawner,
        on_exit=[recorder_node],
    ))

    return LaunchDescription(declared_arguments + [
        control_node,
        robot_state_pub_node,
        jsb_spawner,
        delay_rviz,
        delay_fts,
        delay_admittance,
        delay_recorder,
    ])
