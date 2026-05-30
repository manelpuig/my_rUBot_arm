#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2


def quaternion_from_rpy(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return [qx, qy, qz, qw]


class MoveToPose(Node):

    def __init__(self):
        super().__init__("move_to_pose")

        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("end_effector", "puma_tool")

        self.declare_parameter("target_xyz", [0.4, 0.0, 0.3])
        self.declare_parameter("target_rpy", [0.0, 3.14159, 0.0])

        self.declare_parameter("max_velocity", 0.3)
        self.declare_parameter("max_acceleration", 0.3)
        self.declare_parameter("execute", True)
        self.declare_parameter("print_debug", True)

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.base_link = str(self.get_parameter("base_link").value)
        self.end_effector = str(self.get_parameter("end_effector").value)

        self.target_xyz = list(self.get_parameter("target_xyz").value)
        self.target_rpy = list(self.get_parameter("target_rpy").value)

        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.execute = bool(self.get_parameter("execute").value)
        self.print_debug = bool(self.get_parameter("print_debug").value)

        if len(self.joint_names) == 0:
            raise RuntimeError("Parameter 'joint_names' is empty.")

        if len(self.target_xyz) != 3:
            raise RuntimeError("Parameter 'target_xyz' must have 3 values.")

        if len(self.target_rpy) != 3:
            raise RuntimeError("Parameter 'target_rpy' must have 3 values.")

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name=self.base_link,
            end_effector_name=self.end_effector,
            group_name=self.group_name,
        )

        self.moveit2.max_velocity = self.max_velocity
        self.moveit2.max_acceleration = self.max_acceleration

        self.done = False
        self.timer = self.create_timer(2.0, self.run_once)

    def run_once(self):
        if self.done:
            return

        self.done = True

        position = [
            float(self.target_xyz[0]),
            float(self.target_xyz[1]),
            float(self.target_xyz[2]),
        ]

        quat_xyzw = quaternion_from_rpy(
            float(self.target_rpy[0]),
            float(self.target_rpy[1]),
            float(self.target_rpy[2]),
        )

        if self.print_debug:
            self.get_logger().info(f"Planning group: {self.group_name}")
            self.get_logger().info(f"Base link: {self.base_link}")
            self.get_logger().info(f"End effector: {self.end_effector}")
            self.get_logger().info(f"Target xyz: {position}")
            self.get_logger().info(f"Target rpy: {self.target_rpy}")
            self.get_logger().info(f"Target quaternion xyzw: {quat_xyzw}")

        if not self.execute:
            self.get_logger().info("execute=false -> exiting without motion.")
            rclpy.shutdown()
            return

        self.get_logger().info("Calling move_to_pose()...")

        self.moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=False,
        )

        self.get_logger().info("move_to_pose() returned.")
        self.get_logger().info("Waiting until execution is finished...")

        self.moveit2.wait_until_executed()

        self.get_logger().info("Motion finished.")
        rclpy.shutdown()


def main():
    rclpy.init()

    node = MoveToPose()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()