#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SendJointTargetNode(Node):

    def __init__(self):
        super().__init__("send_joint_target_node")

        self.declare_parameter(
            "topic_name",
            "/arm_controller/joint_trajectory"
        )

        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        )

        self.declare_parameter(
            "target_joints_deg",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )

        self.declare_parameter("duration", 2.0)

        topic_name = self.get_parameter("topic_name").value

        self.publisher = self.create_publisher(
            JointTrajectory,
            topic_name,
            10
        )

        self.timer = self.create_timer(1.0, self.publish_target)
        self.has_published = False

        self.get_logger().info(
            f"Send joint target node ready. Topic: {topic_name}"
        )

    def publish_target(self):
        if self.has_published:
            return

        joint_names = list(self.get_parameter("joint_names").value)
        target_joints_deg = list(self.get_parameter("target_joints_deg").value)
        duration = float(self.get_parameter("duration").value)

        if len(target_joints_deg) != len(joint_names):
            self.get_logger().error(
                f"target_joints_deg has {len(target_joints_deg)} values, "
                f"but joint_names has {len(joint_names)} joints"
            )
            return

        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [
            math.radians(angle_deg)
            for angle_deg in target_joints_deg
        ]

        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int(
            (duration - int(duration)) * 1e9
        )

        trajectory.points.append(point)

        self.publisher.publish(trajectory)

        self.get_logger().info(
            f"Published joint target [deg]: {target_joints_deg}"
        )
        self.get_logger().info(
            f"Published joint target [rad]: "
            f"{[round(q, 4) for q in point.positions]}"
        )
        self.get_logger().info(
            f"Duration: {duration:.2f} s"
        )

        self.has_published = True


def main(args=None):
    rclpy.init(args=args)
    node = SendJointTargetNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()