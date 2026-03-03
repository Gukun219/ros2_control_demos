:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_16/doc/userdoc.rst

.. _ros2_control_demos_example_16_userdoc:

Example 16: 6-DOF robot with admittance controller
===================================================

This example demonstrates the use of the ``admittance_controller`` from
``ros2_controllers`` with a 6-DOF robot (r6bot) that includes a force-torque
sensor at the end-effector.

The admittance controller implements a spring-mass-damper system in Cartesian
space, allowing the robot to comply with external forces while tracking a
desired target pose.

Architecture
------------

The demo uses the following components:

* **Hardware Interface** (``ros2_control_demo_example_16/RobotSystem``):
  A simulated 6-DOF robot with position/velocity command interfaces and a
  force-torque sensor that generates sinusoidal force readings for
  demonstration purposes.

* **Admittance Controller** (``admittance_controller/AdmittanceController``):
  Computes position/velocity commands based on the difference between
  desired and actual Cartesian pose, modulated by the measured
  force/torque through admittance dynamics.

* **Force-Torque Sensor Broadcaster**
  (``force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster``):
  Publishes force-torque sensor readings as ``geometry_msgs/WrenchStamped``
  messages.

* **Joint State Broadcaster**
  (``joint_state_broadcaster/JointStateBroadcaster``):
  Publishes joint states for visualization.

* **Reference Generator** (``send_target_pose``):
  Publishes a circular trajectory of target poses for the admittance
  controller to follow.

Tutorial steps
--------------

1. To check that the r6bot description is working properly, use:

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_16 view_r6bot.launch.py

2. To start the example with the admittance controller, open a terminal and run:

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_16 r6bot_admittance_controller.launch.py

3. Check if the controllers are running:

   .. code-block:: shell

    ros2 control list_controllers

   You should see the following controllers active:

   * ``joint_state_broadcaster``
   * ``force_torque_sensor_broadcaster``
   * ``admittance_controller``

4. To view the force-torque sensor data:

   .. code-block:: shell

    ros2 topic echo /force_torque_sensor_broadcaster/wrench

5. To send a target pose trajectory, open another terminal and run:

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_16 send_target_pose.launch.py

6. To dynamically adjust admittance parameters (e.g., stiffness):

   .. code-block:: shell

    ros2 param set /admittance_controller admittance.stiffness "[100.0, 100.0, 100.0, 10.0, 10.0, 10.0]"

Configuration
-------------

The admittance controller is configured via
``bringup/config/r6bot_admittance_controller.yaml``. Key parameters include:

* ``admittance.mass``: Virtual mass for each Cartesian axis
* ``admittance.damping_ratio``: Damping ratio (relative to critical damping)
* ``admittance.stiffness``: Virtual stiffness for each axis
* ``admittance.selected_axes``: Enable/disable admittance on specific axes
* ``kinematics.plugin_name``: Kinematics solver plugin (KDL by default)
* ``ft_sensor.name``: Name of the force-torque sensor in the hardware description
