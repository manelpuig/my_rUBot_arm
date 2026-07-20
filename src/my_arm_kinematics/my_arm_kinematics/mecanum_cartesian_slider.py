#!/usr/bin/env python3

import math
import tkinter as tk

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def ikine_wrist_position(x, y, z, z_shoulder, l2, l3, elbow):
    """Analytical IK for the joint4 origin of the mecanum servo arm.

    Args:
        x, y, z: Desired wrist-centre position in metres, relative to base_link.
        z_shoulder: Height of joint2 relative to base_link, in metres.
        l2, l3: Upper-arm and forearm lengths, in metres.
        elbow: Either "up" or "down".

    Returns:
        [q1, q2, q3] in radians.

    The signs follow the axes used by my_arm_mecanum.urdf.xacro. A negative
    q2 raises link2 above its horizontal zero configuration.
    """
    radial = math.hypot(x, y)
    height = z - z_shoulder

    if radial < 1.0e-6:
        raise ValueError("Target is too close to the joint1 axis")

    q1 = math.atan2(y, x)

    cos_q3 = (
        radial * radial
        + height * height
        - l2 * l2
        - l3 * l3
    ) / (2.0 * l2 * l3)

    tolerance = 1.0e-9
    if cos_q3 < -1.0 - tolerance or cos_q3 > 1.0 + tolerance:
        raise ValueError("Wrist target is outside the reachable workspace")

    cos_q3 = max(-1.0, min(1.0, cos_q3))
    q3_abs = math.acos(cos_q3)

    if elbow == "up":
        q3 = q3_abs
    elif elbow == "down":
        q3 = -q3_abs
    else:
        raise ValueError("elbow must be 'up' or 'down'")

    q2 = math.atan2(-height, radial) - math.atan2(
        l3 * math.sin(q3),
        l2 + l3 * math.cos(q3),
    )

    return [q1, q2, q3]


class MecanumCartesianSlider(Node):
    """GUI for wrist-position control of the mecanum servo arm."""

    def __init__(self):
        super().__init__("mecanum_cartesian_slider")

        self.declare_parameter(
            "controller_topic", "/arm_controller/joint_trajectory"
        )
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )

        # Geometry from my_arm_mecanum.urdf.xacro.
        self.declare_parameter("base_z", 0.030)
        self.declare_parameter("L1", 0.055)
        self.declare_parameter("L2", 0.075)
        self.declare_parameter("L3", 0.075)

        self.declare_parameter("initial_wrist_xyz_mm", [120.0, 0.0, 100.0])
        self.declare_parameter("x_limits_mm", [30.0, 150.0])
        self.declare_parameter("y_limits_mm", [-120.0, 120.0])
        self.declare_parameter("z_limits_mm", [20.0, 220.0])
        self.declare_parameter("q4_limits_deg", [-90.0, 90.0])
        self.declare_parameter("q5_limits_deg", [-90.0, 90.0])
        self.declare_parameter("initial_q4_deg", 0.0)
        self.declare_parameter("initial_q5_deg", 0.0)

        self.declare_parameter("elbow", "up")
        self.declare_parameter("joint_min_deg", [-90.0] * 6)
        self.declare_parameter("joint_max_deg", [90.0] * 6)

        # Temporary physical-driver convention for joint6. These values are
        # relative servo angles and must be calibrated on the real gripper.
        self.declare_parameter("gripper_open_joint_deg", 0.0)
        self.declare_parameter("gripper_closed_joint_deg", 45.0)

        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("duration", 0.0)
        self.declare_parameter("live_send_on_start", True)

        self.controller_topic = self.get_parameter("controller_topic").value
        self.joint_names = list(self.get_parameter("joint_names").value)

        self.base_z = float(self.get_parameter("base_z").value)
        self.l1 = float(self.get_parameter("L1").value)
        self.l2 = float(self.get_parameter("L2").value)
        self.l3 = float(self.get_parameter("L3").value)
        self.z_shoulder = self.base_z + self.l1

        self.initial_xyz_mm = list(
            self.get_parameter("initial_wrist_xyz_mm").value
        )
        self.x_limits_mm = list(self.get_parameter("x_limits_mm").value)
        self.y_limits_mm = list(self.get_parameter("y_limits_mm").value)
        self.z_limits_mm = list(self.get_parameter("z_limits_mm").value)
        self.q4_limits_deg = list(
            self.get_parameter("q4_limits_deg").value
        )
        self.q5_limits_deg = list(
            self.get_parameter("q5_limits_deg").value
        )
        self.initial_q4_deg = float(
            self.get_parameter("initial_q4_deg").value
        )
        self.initial_q5_deg = float(
            self.get_parameter("initial_q5_deg").value
        )

        self.default_elbow = str(self.get_parameter("elbow").value).lower()
        self.joint_min_deg = list(self.get_parameter("joint_min_deg").value)
        self.joint_max_deg = list(self.get_parameter("joint_max_deg").value)
        self.gripper_open_deg = float(
            self.get_parameter("gripper_open_joint_deg").value
        )
        self.gripper_closed_deg = float(
            self.get_parameter("gripper_closed_joint_deg").value
        )

        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        self.duration = float(self.get_parameter("duration").value)
        self.live_send_on_start = bool(
            self.get_parameter("live_send_on_start").value
        )

        self._validate_parameters()

        self.publisher = self.create_publisher(
            JointTrajectory, self.controller_topic, 10
        )

        self.root = tk.Tk()
        self.root.title("rUBot mecanum Cartesian arm control")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.live_send = tk.BooleanVar(value=self.live_send_on_start)
        self.elbow = tk.StringVar(value=self.default_elbow)
        self.gripper = tk.StringVar(value="open")
        self.running = True
        self.target_dirty = False

        self._build_gui()
        # Creating and setting the scales must not command the real robot.
        self.target_dirty = False

        self.period_ms = max(
            1, int(round(1000.0 / self.publish_rate_hz))
        )
        self.root.after(self.period_ms, self._publish_loop)

        self.get_logger().info(
            f"Cartesian slider ready. Wrist targets will be published to "
            f"{self.controller_topic} at up to {self.publish_rate_hz:.1f} Hz"
        )

    def _validate_parameters(self):
        if len(self.joint_names) != 6:
            raise ValueError("joint_names must contain exactly 6 values")

        arrays_with_size = {
            "initial_wrist_xyz_mm": (self.initial_xyz_mm, 3),
            "x_limits_mm": (self.x_limits_mm, 2),
            "y_limits_mm": (self.y_limits_mm, 2),
            "z_limits_mm": (self.z_limits_mm, 2),
            "q4_limits_deg": (self.q4_limits_deg, 2),
            "q5_limits_deg": (self.q5_limits_deg, 2),
            "joint_min_deg": (self.joint_min_deg, 6),
            "joint_max_deg": (self.joint_max_deg, 6),
        }
        for name, (values, expected_size) in arrays_with_size.items():
            if len(values) != expected_size:
                raise ValueError(
                    f"{name} must contain exactly {expected_size} values"
                )

        if min(self.l1, self.l2, self.l3) <= 0.0:
            raise ValueError("L1, L2 and L3 must be greater than zero")
        if self.default_elbow not in ("up", "down"):
            raise ValueError("elbow must be 'up' or 'down'")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")
        if self.duration < 0.0:
            raise ValueError("duration cannot be negative")

    def _make_scale(self, row, label, limits, initial_value):
        tk.Label(self.root, text=label).grid(
            row=row, column=0, padx=(12, 6), sticky="w"
        )
        scale = tk.Scale(
            self.root,
            from_=float(limits[0]),
            to=float(limits[1]),
            resolution=1.0,
            orient=tk.HORIZONTAL,
            length=380,
            command=self._target_changed,
        )
        scale.set(float(initial_value))
        scale.grid(row=row, column=1, columnspan=3, padx=(0, 12))
        return scale

    def _build_gui(self):
        tk.Label(
            self.root,
            text="Wrist-centre target",
            font=("TkDefaultFont", 12, "bold"),
        ).grid(row=0, column=0, columnspan=4, padx=12, pady=(12, 2))

        tk.Label(
            self.root,
            text="x, y, z are the position of joint4 relative to base_link",
        ).grid(row=1, column=0, columnspan=4, padx=12, pady=(0, 6))

        self.x_slider = self._make_scale(
            2, "x [mm]", self.x_limits_mm, self.initial_xyz_mm[0]
        )
        self.y_slider = self._make_scale(
            3, "y [mm]", self.y_limits_mm, self.initial_xyz_mm[1]
        )
        self.z_slider = self._make_scale(
            4, "z [mm]", self.z_limits_mm, self.initial_xyz_mm[2]
        )
        self.q4_slider = self._make_scale(
            5, "joint4 [deg]", self.q4_limits_deg, self.initial_q4_deg
        )
        self.q5_slider = self._make_scale(
            6, "joint5 [deg]", self.q5_limits_deg, self.initial_q5_deg
        )

        tk.Label(self.root, text="Elbow").grid(
            row=7, column=0, padx=(12, 6), pady=8, sticky="w"
        )
        tk.OptionMenu(
            self.root,
            self.elbow,
            "up",
            "down",
            command=self._target_changed,
        ).grid(row=7, column=1, pady=8, sticky="w")

        tk.Label(self.root, text="Gripper").grid(
            row=7, column=2, padx=(12, 2), pady=8, sticky="e"
        )
        gripper_frame = tk.Frame(self.root)
        gripper_frame.grid(row=7, column=3, padx=(0, 12), pady=8)
        tk.Radiobutton(
            gripper_frame,
            text="Open",
            variable=self.gripper,
            value="open",
            command=self._target_changed,
        ).pack(side=tk.LEFT)
        tk.Radiobutton(
            gripper_frame,
            text="Closed",
            variable=self.gripper,
            value="closed",
            command=self._target_changed,
        ).pack(side=tk.LEFT)

        tk.Checkbutton(
            self.root,
            text="Live send",
            variable=self.live_send,
        ).grid(row=8, column=0, padx=12, pady=8, sticky="w")
        tk.Button(
            self.root,
            text="Send target",
            command=self.publish_target,
            width=14,
        ).grid(row=8, column=1, pady=8)
        tk.Button(
            self.root,
            text="Reset GUI",
            command=self._reset_gui,
            width=14,
        ).grid(row=8, column=2, columnspan=2, padx=(0, 12), pady=8)

        self.joint_solution_label = tk.Label(
            self.root,
            text="IK solution: q1 = --, q2 = --, q3 = --",
            anchor="w",
        )
        self.joint_solution_label.grid(
            row=9, column=0, columnspan=4, padx=12, pady=(2, 2), sticky="we"
        )

        self.status_label = tk.Label(
            self.root,
            text="Move a slider or press Send target",
            anchor="w",
        )
        self.status_label.grid(
            row=10,
            column=0,
            columnspan=4,
            padx=12,
            pady=(2, 12),
            sticky="we",
        )

    def _target_changed(self, _value=None):
        self.target_dirty = True

    def _reset_gui(self):
        self.x_slider.set(self.initial_xyz_mm[0])
        self.y_slider.set(self.initial_xyz_mm[1])
        self.z_slider.set(self.initial_xyz_mm[2])
        self.q4_slider.set(self.initial_q4_deg)
        self.q5_slider.set(self.initial_q5_deg)
        self.elbow.set(self.default_elbow)
        self.gripper.set("open")
        self.target_dirty = True

    def _current_joint_target(self):
        x = float(self.x_slider.get()) / 1000.0
        y = float(self.y_slider.get()) / 1000.0
        z = float(self.z_slider.get()) / 1000.0

        q1, q2, q3 = ikine_wrist_position(
            x,
            y,
            z,
            self.z_shoulder,
            self.l2,
            self.l3,
            self.elbow.get(),
        )

        q4 = math.radians(float(self.q4_slider.get()))
        q5 = math.radians(float(self.q5_slider.get()))
        q6_deg = (
            self.gripper_open_deg
            if self.gripper.get() == "open"
            else self.gripper_closed_deg
        )
        q6 = math.radians(q6_deg)

        joint_target = [q1, q2, q3, q4, q5, q6]
        joint_target_deg = [math.degrees(value) for value in joint_target]

        for index, value_deg in enumerate(joint_target_deg):
            if not (
                self.joint_min_deg[index]
                <= value_deg
                <= self.joint_max_deg[index]
            ):
                raise ValueError(
                    f"{self.joint_names[index]}={value_deg:.1f} deg is outside "
                    f"[{self.joint_min_deg[index]:.1f}, "
                    f"{self.joint_max_deg[index]:.1f}] deg"
                )

        return joint_target, joint_target_deg

    def _publish_loop(self):
        if not self.running:
            return

        if self.live_send.get() and self.target_dirty:
            self.publish_target()

        self.root.after(self.period_ms, self._publish_loop)

    def publish_target(self):
        self.target_dirty = False

        try:
            joint_target, joint_target_deg = self._current_joint_target()
        except ValueError as error:
            self.joint_solution_label.config(
                text="IK solution: q1 = --, q2 = --, q3 = --"
            )
            self.status_label.config(text=str(error), fg="red")
            return

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_target
        point.time_from_start.sec = int(self.duration)
        point.time_from_start.nanosec = int(
            (self.duration - int(self.duration)) * 1.0e9
        )
        trajectory.points.append(point)

        self.publisher.publish(trajectory)

        self.joint_solution_label.config(
            text=(
                f"IK solution: q1 = {joint_target_deg[0]:.1f}, "
                f"q2 = {joint_target_deg[1]:.1f}, "
                f"q3 = {joint_target_deg[2]:.1f} deg"
            )
        )
        self.status_label.config(
            text=(
                f"Target sent; q4 = {joint_target_deg[3]:.1f}, "
                f"q5 = {joint_target_deg[4]:.1f}, "
                f"gripper = {self.gripper.get()}"
            ),
            fg="darkgreen",
        )

    def run(self):
        self.root.mainloop()

    def close(self):
        self.running = False
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumCartesianSlider()

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
