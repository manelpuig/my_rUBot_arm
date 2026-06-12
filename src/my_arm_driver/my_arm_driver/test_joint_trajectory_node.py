#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TestJointTrajectoryNode(Node):

    def __init__(self):
        super().__init__("test_joint_trajectory_node")

        self.declare_parameter(
            "topic_name",
            "/arm_controller/joint_trajectory"
        )

        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        )

        self.declare_parameter("joint_index", 0)
        self.declare_parameter("amplitude_deg", 20.0)
        self.declare_parameter("step_time", 1.0)

        topic_name = self.get_parameter("topic_name").value

        self.publisher = self.create_publisher(
            JointTrajectory,
            topic_name,
            10
        )

        self.timer = self.create_timer(1.0, self.publish_test_trajectory)
        self.has_published = False

        self.get_logger().info(
            f"Test JointTrajectory publisher ready. Topic: {topic_name}"
        )

    def make_point(self, positions_deg, time_from_start_sec):
        point = JointTrajectoryPoint()

        point.positions = [
            math.radians(angle_deg) for angle_deg in positions_deg
        ]

        point.time_from_start.sec = int(time_from_start_sec)
        point.time_from_start.nanosec = int(
            (time_from_start_sec - int(time_from_start_sec)) * 1e9
        )

        return point

    def publish_test_trajectory(self):
        if self.has_published:
            return

        joint_names = list(self.get_parameter("joint_names").value)
        joint_index = int(self.get_parameter("joint_index").value)
        amplitude_deg = float(self.get_parameter("amplitude_deg").value)
        step_time = float(self.get_parameter("step_time").value)

        n_joints = len(joint_names)

        if joint_index < 0 or joint_index >= n_joints:
            self.get_logger().error(
                f"Invalid joint_index={joint_index}. "
                f"Valid range is 0 to {n_joints - 1}"
            )
            return

        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        p0 = [0.0] * n_joints
        p1 = [0.0] * n_joints
        p2 = [0.0] * n_joints
        p3 = [0.0] * n_joints
        p4 = [0.0] * n_joints

        p1[joint_index] = amplitude_deg * 0.5
        p2[joint_index] = amplitude_deg
        p3[joint_index] = amplitude_deg * 0.5

        trajectory.points = [
            self.make_point(p0, 0.0),
            self.make_point(p1, step_time),
            self.make_point(p2, 2.0 * step_time),
            self.make_point(p3, 3.0 * step_time),
            self.make_point(p4, 4.0 * step_time),
        ]

        self.publisher.publish(trajectory)

        self.get_logger().info("Published test JointTrajectory")
        self.get_logger().info(f"Joint names: {joint_names}")
        self.get_logger().info(f"Moving joint index: {joint_index}")
        self.get_logger().info(f"Amplitude: {amplitude_deg} deg")
        self.get_logger().info(f"Step time: {step_time} s")

        self.has_published = True


def main(args=None):
    rclpy.init(args=args)
    node = TestJointTrajectoryNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()