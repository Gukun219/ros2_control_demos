# ros2_control_demo_example_16

Demo for a 6 DOF robot with admittance controller using ros2_control.

It consists of the following:
* bringup: launch files and admittance controller configuration
* description: the 6-DOF robot description with force-torque sensor
* hardware: ros2_control hardware interface with simulated F/T sensor
* reference_generator: A target pose generator for the admittance controller

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org).

# 演示
## terminal 1
colcon build --packages-select ros2_control_demo_example_16
source install/setup.bash
ros2 launch ros2_control_demo_example_16 r6bot_comparison_demo.launch.py
## terminal 2
source install/setup.bash
ros2 launch ros2_control_demo_example_16 send_target_pose.launch.py
## terminal 3
source install/setup.bash
ros2 param set /admittance_controller admittance.stiffness "[200.0, 200.0, 200.0, 20.0, 20.0, 20.0]"
ros2 param set /admittance_controller admittance.stiffness "[10000.0, 10000.0, 10000.0, 1000.0, 1000.0, 1000.0]"