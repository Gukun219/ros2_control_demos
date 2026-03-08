#!/usr/bin/env python3
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
Admittance Controller Data Recorder

Continuously records end-effector position (via TF) and F/T sensor data.
You manually switch stiffness in another terminal to see the difference.
Press Ctrl+C to stop — a time-series plot is generated automatically.

Usage:
  # Terminal 1: start robot + admittance controller
  ros2 launch ros2_control_demo_example_16 r6bot_admittance_controller.launch.py

  # Terminal 2: start recorder
  ros2 run ros2_control_demo_example_16 record_and_compare

  # Terminal 3: manually switch stiffness
  ros2 param set /admittance_controller admittance.stiffness \
      "[10000.0, 10000.0, 10000.0, 1000.0, 1000.0, 1000.0]"
  ros2 param set /admittance_controller admittance.stiffness \
      "[200.0, 200.0, 200.0, 20.0, 20.0, 20.0]"

  # Press Ctrl+C in Terminal 2 to stop and generate plot
"""

import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class DataRecorder(Node):
    def __init__(self):
        super().__init__("data_recorder")

        self.declare_parameter("output_dir", "/tmp/admittance_comparison")
        self.output_dir = self.get_parameter("output_dir").value
        os.makedirs(self.output_dir, exist_ok=True)

        # TF2 for end-effector position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # F/T sensor data
        self.current_wrench = [0.0, 0.0, 0.0]
        self.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster/wrench",
            self._wrench_cb,
            10,
        )

        # Data: each entry = [t, x, y, z, fx, fy, fz]
        self.data = []
        self.start_time = time.time()

        self.timer = self.create_timer(0.02, self._tick)  # 50 Hz
        self.get_logger().info("Data recorder started. Recording continuously...")
        self.get_logger().info(f"Output dir: {self.output_dir}")
        self.get_logger().info("Press Ctrl+C to stop and generate plot.")

    def _wrench_cb(self, msg):
        self.current_wrench = [
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
        ]

    def _tick(self):
        try:
            t = self.tf_buffer.lookup_transform("base_link", "tool0", Time())
            elapsed = time.time() - self.start_time
            self.data.append([
                elapsed,
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
                self.current_wrench[0],
                self.current_wrench[1],
                self.current_wrench[2],
            ])
        except TransformException:
            pass

    def generate_plot(self):
        """Called on shutdown to generate the comparison plot."""
        if len(self.data) < 10:
            self.get_logger().error("Not enough data to plot.")
            return

        d = np.array(self.data)
        # Columns: [t, x, y, z, fx, fy, fz]

        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        fig.suptitle(
            "Admittance Controller Recording\n"
            "Switch stiffness manually to see the difference",
            fontsize=14, fontweight="bold",
        )

        # --- Plot 1: End-effector XYZ position ---
        ax = axes[0]
        ax.plot(d[:, 0], d[:, 1], "r-", linewidth=1.0, label="X")
        ax.plot(d[:, 0], d[:, 2], "g-", linewidth=1.0, label="Y")
        ax.plot(d[:, 0], d[:, 3], "b-", linewidth=1.5, label="Z")
        ax.set_ylabel("Position (m)")
        ax.set_title("End-Effector Position (tool0 in base_link)")
        ax.legend()
        ax.grid(True, alpha=0.3)

        # --- Plot 2: Z position (main comparison axis) ---
        ax = axes[1]
        ax.plot(d[:, 0], d[:, 3], "b-", linewidth=1.5, label="Z position")
        ax.set_ylabel("Z (m)")
        ax.set_title("Z Position — compare amplitude before/after stiffness change")
        ax.legend()
        ax.grid(True, alpha=0.3)

        # --- Plot 3: Force Z ---
        ax = axes[2]
        ax.plot(d[:, 0], d[:, 6], "m-", linewidth=1.0, label="Force Z")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Force (N)")
        ax.set_title("Applied Force on Z-axis (simulated)")
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()

        plot_path = os.path.join(self.output_dir, "admittance_recording.png")
        fig.savefig(plot_path, dpi=150)
        plt.close(fig)
        self.get_logger().info(f"Plot saved to: {plot_path}")

        # Save CSV
        csv_path = os.path.join(self.output_dir, "recording.csv")
        header = "time,x,y,z,force_x,force_y,force_z"
        np.savetxt(csv_path, d, delimiter=",", header=header, comments="")
        self.get_logger().info(f"Data saved to: {csv_path}")
        self.get_logger().info(f"Recorded {len(self.data)} points over {d[-1, 0]:.1f}s")


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Stopping recorder, generating plot...")
        node.generate_plot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
