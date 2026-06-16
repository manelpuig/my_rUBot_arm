#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SendJointTrajectoryNode(Node):

    def __init__(self):
        super().__init__("send_joint_trajectory_node")

        self.declare_parameter(
            "topic_name",
            "/arm_controller/joint_trajectory"
        )

        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        )

        # Several target points in degrees.
        # Each inner list is one trajectory point:
        # [joint1, joint2, joint3, joint4, joint5, joint6]
        self.declare_parameter(
            "trajectory_points_deg",
            [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [20.0, 20.0, 0.0, 0.0, 0.0, 0.0],
                [20.0, 20.0, -20.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ]
        )

        # Absolute time_from_start for each point, in seconds.
        self.declare_parameter(
            "point_times_sec",
            [0.0, 1.0, 2.0, 3.0, 4.0]
        )

        topic_name = self.get_parameter("topic_name").value

        self.publisher = self.create_publisher(
            JointTrajectory,
            topic_name,
            10
        )

        self.timer = self.create_timer(1.0, self.publish_trajectory)
        self.has_published = False

        self.get_logger().info(
            f"Send joint trajectory node ready. Topic: {topic_name}"
        )

    def seconds_to_duration_msg(self, t):
        sec = int(t)
        nanosec = int((t - sec) * 1e9)
        return sec, nanosec

    def publish_trajectory(self):
        if self.has_published:
            return

        joint_names = list(self.get_parameter("joint_names").value)
        trajectory_points_deg = list(
            self.get_parameter("trajectory_points_deg").value
        )
        point_times_sec = list(
            self.get_parameter("point_times_sec").value
        )

        if len(trajectory_points_deg) != len(point_times_sec):
            self.get_logger().error(
                "trajectory_points_deg and point_times_sec must have "
                "the same number of elements"
            )
            return

        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        for i, target_deg in enumerate(trajectory_points_deg):

            target_deg = list(target_deg)

            if len(target_deg) != len(joint_names):
                self.get_logger().error(
                    f"Point {i} has {len(target_deg)} values, "
                    f"but joint_names has {len(joint_names)} joints"
                )
                return

            point = JointTrajectoryPoint()

            point.positions = [
                math.radians(angle_deg)
                for angle_deg in target_deg
            ]

            sec, nanosec = self.seconds_to_duration_msg(
                float(point_times_sec[i])
            )

            point.time_from_start.sec = sec
            point.time_from_start.nanosec = nanosec

            trajectory.points.append(point)

        self.publisher.publish(trajectory)

        self.get_logger().info(
            f"Published trajectory with {len(trajectory.points)} points"
        )

        for i, point in enumerate(trajectory.points):
            self.get_logger().info(
                f"Point {i}: "
                f"t={point_times_sec[i]:.2f}s | "
                f"joints [deg]={trajectory_points_deg[i]}"
            )

        self.has_published = True


def main(args=None):
    rclpy.init(args=args)
    node = SendJointTrajectoryNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()