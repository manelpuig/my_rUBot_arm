#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def clamp(x, min_value=-1.0, max_value=1.0):
    return max(min(x, max_value), min_value)


def ikine_puma_xyz_simple(x, y, z):
    """
    Simple geometric IK for the first 3 joints of a PUMA-like arm.

    This version computes q1, q2, q3 from XYZ.
    Wrist joints are assigned simply for didactic purposes.

    Units:
        x, y, z in meters
        output joints in radians
    """

    # Approximate PUMA dimensions
    d1 = 0.0
    a2 = 0.4318
    d4 = 0.4318

    # Joint 1
    q1 = math.atan2(y, x)

    # Planar distance from base axis
    r = math.sqrt(x**2 + y**2)
    z_eff = z - d1

    # Distance shoulder -> wrist approximation
    D = math.sqrt(r**2 + z_eff**2)

    # Check reachability
    if D > (a2 + d4):
        raise RuntimeError("Target is outside the robot workspace")

    if D < abs(a2 - d4):
        raise RuntimeError("Target is too close to the robot base")

    # Law of cosines
    cos_q3 = clamp((D**2 - a2**2 - d4**2) / (2.0 * a2 * d4))
    q3 = math.acos(cos_q3)

    alpha = math.atan2(z_eff, r)
    beta = math.atan2(
        d4 * math.sin(q3),
        a2 + d4 * math.cos(q3)
    )

    q2 = alpha - beta

    # Simple wrist assignment
    q4 = 0.0
    q5 = 0.0
    q6 = 0.0

    return [q1, q2, q3, q4, q5, q6]


class PumaIKine(Node):

    def __init__(self):
        super().__init__("puma_ikine")

        self.declare_parameter("target_xyz", [0.5422, 0.1397, 0.9423])
        self.declare_parameter("time_sec", 5.0)
        self.declare_parameter(
            "controller_topic",
            "/arm_controller/joint_trajectory"
        )

        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        )

        self.target_xyz = list(self.get_parameter("target_xyz").value)
        self.time_sec = float(self.get_parameter("time_sec").value)
        self.controller_topic = self.get_parameter("controller_topic").value
        self.joint_names = list(self.get_parameter("joint_names").value)

        if len(self.target_xyz) != 3:
            raise RuntimeError("target_xyz must contain exactly 3 values")

        self.pub = self.create_publisher(
            JointTrajectory,
            self.controller_topic,
            10
        )

        self.sent = False
        self.done = False

        self.timer = self.create_timer(1.0, self.compute_and_send)

        self.get_logger().info(f"Publishing to: {self.controller_topic}")

    def compute_and_send(self):
        if self.sent:
            return

        x, y, z = self.target_xyz

        try:
            q_rad = ikine_puma_xyz_simple(x, y, z)
        except RuntimeError as e:
            self.get_logger().error(str(e))
            self.done = True
            return

        q_deg = [math.degrees(q) for q in q_rad]

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = q_rad

        secs = int(self.time_sec)
        nsecs = int((self.time_sec - secs) * 1e9)
        point.time_from_start = Duration(sec=secs, nanosec=nsecs)

        msg.points.append(point)

        self.pub.publish(msg)

        self.get_logger().info(f"Target XYZ [m]: {self.target_xyz}")
        self.get_logger().info(f"IK solution [deg]: {q_deg}")
        self.get_logger().info(f"IK solution [rad]: {q_rad}")

        self.sent = True
        self.timer.cancel()

        self.finish_timer = self.create_timer(
            self.time_sec + 0.5,
            self.finish_node
        )

    def finish_node(self):
        self.get_logger().info("Motion finished. Closing node.")
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = PumaIKine()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()