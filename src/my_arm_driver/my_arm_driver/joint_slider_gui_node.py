#!/usr/bin/env python3

import math
import tkinter as tk

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointSliderGuiNode(Node):
    """Simple GUI for manually commanding the six servo-arm joints."""

    def __init__(self):
        super().__init__("joint_slider_gui_node")

        self.declare_parameter(
            "topic_name", "/arm_controller/joint_trajectory"
        )
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter("initial_joints_deg", [0.0] * 6)
        self.declare_parameter("joint_min_deg", [-90, -90, -90, -90, -90, 0])
        self.declare_parameter("joint_max_deg", [ 90,  90,  90,  90,  90, 45])
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("duration", 0.0)

        self.topic_name = self.get_parameter("topic_name").value
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.initial_joints_deg = list(
            self.get_parameter("initial_joints_deg").value
        )
        self.joint_min_deg = list(self.get_parameter("joint_min_deg").value)
        self.joint_max_deg = list(self.get_parameter("joint_max_deg").value)
        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        self.duration = float(self.get_parameter("duration").value)

        self._validate_parameters()

        self.publisher = self.create_publisher(
            JointTrajectory, self.topic_name, 10
        )

        self.root = tk.Tk()
        self.root.title("rUBot arm joint control")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.live_send = tk.BooleanVar(value=True)
        self.sliders = []
        self.target_dirty = False
        self.running = True

        self._build_gui()

        period_ms = max(1, int(round(1000.0 / self.publish_rate_hz)))
        self.root.after(period_ms, self._publish_loop)

        self.get_logger().info(
            f"Joint slider GUI ready. Publishing to {self.topic_name} "
            f"at up to {self.publish_rate_hz:.1f} Hz"
        )

    def _validate_parameters(self):
        count = len(self.joint_names)
        arrays = {
            "initial_joints_deg": self.initial_joints_deg,
            "joint_min_deg": self.joint_min_deg,
            "joint_max_deg": self.joint_max_deg,
        }

        if count != 6:
            raise ValueError("joint_names must contain exactly 6 joints")

        for name, values in arrays.items():
            if len(values) != count:
                raise ValueError(
                    f"{name} must contain {count} values, got {len(values)}"
                )

        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")

        if self.duration < 0.0:
            raise ValueError("duration cannot be negative")

    def _build_gui(self):
        title = tk.Label(
            self.root,
            text="Joint targets [degrees]",
            font=("TkDefaultFont", 12, "bold"),
        )
        title.grid(row=0, column=0, columnspan=3, padx=12, pady=(12, 6))

        for index, joint_name in enumerate(self.joint_names):
            tk.Label(self.root, text=joint_name).grid(
                row=index + 1, column=0, padx=(12, 6), sticky="w"
            )

            slider = tk.Scale(
                self.root,
                from_=self.joint_min_deg[index],
                to=self.joint_max_deg[index],
                resolution=1.0,
                orient=tk.HORIZONTAL,
                length=360,
                command=self._slider_changed,
            )
            slider.set(self.initial_joints_deg[index])
            slider.grid(row=index + 1, column=1, columnspan=2, padx=(0, 12))
            self.sliders.append(slider)

        tk.Checkbutton(
            self.root,
            text="Live send",
            variable=self.live_send,
        ).grid(row=7, column=0, padx=12, pady=10, sticky="w")

        tk.Button(
            self.root,
            text="Send target",
            command=self.publish_target,
            width=14,
        ).grid(row=7, column=1, pady=10)

        tk.Button(
            self.root,
            text="Set all to zero",
            command=self._set_zero,
            width=14,
        ).grid(row=7, column=2, padx=(0, 12), pady=10)

        self.status = tk.Label(
            self.root,
            text="Move a slider or press Send target",
            anchor="w",
        )
        self.status.grid(
            row=8, column=0, columnspan=3, padx=12, pady=(0, 12), sticky="we"
        )

    def _slider_changed(self, _value):
        self.target_dirty = True

    def _set_zero(self):
        for slider in self.sliders:
            slider.set(0.0)
        self.target_dirty = True

    def _current_target_deg(self):
        return [float(slider.get()) for slider in self.sliders]

    def _publish_loop(self):
        if not self.running:
            return

        if self.live_send.get() and self.target_dirty:
            self.publish_target()

        period_ms = max(1, int(round(1000.0 / self.publish_rate_hz)))
        self.root.after(period_ms, self._publish_loop)

    def publish_target(self):
        target_deg = self._current_target_deg()

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [math.radians(value) for value in target_deg]
        point.time_from_start.sec = int(self.duration)
        point.time_from_start.nanosec = int(
            (self.duration - int(self.duration)) * 1e9
        )
        trajectory.points.append(point)

        self.publisher.publish(trajectory)
        self.target_dirty = False

        shown = ", ".join(f"{value:.0f}" for value in target_deg)
        self.status.config(text=f"Sent: [{shown}] deg")
        self.get_logger().info(f"Published joint target [deg]: {target_deg}")

    def run(self):
        self.root.mainloop()

    def close(self):
        self.running = False
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = JointSliderGuiNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
