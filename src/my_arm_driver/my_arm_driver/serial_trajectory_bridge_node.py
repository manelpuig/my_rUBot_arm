#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import serial


class SerialTrajectoryBridgeNode(Node):

    def __init__(self):
        super().__init__("serial_trajectory_bridge_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self.declare_parameter("servo_center_deg", [90, 90, 90, 90, 90, 90])
        self.declare_parameter("servo_sign", [1, 1, 1, 1, 1, 1])
        self.declare_parameter("servo_min_deg", [0, 0, 0, 0, 0, 0])
        self.declare_parameter("servo_max_deg", [180, 180, 180, 180, 180, 180])

        self.declare_parameter(
            "initial_joints_deg",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self.declare_parameter("send_initial_position", False)

        self.declare_parameter("joint_names", [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ])

        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("joint_state_rate", 20.0)

        serial_port = self.get_parameter("serial_port").value
        baudrate = int(self.get_parameter("baudrate").value)

        self.servo_center_deg = list(self.get_parameter("servo_center_deg").value)
        self.servo_sign = list(self.get_parameter("servo_sign").value)
        self.servo_min_deg = list(self.get_parameter("servo_min_deg").value)
        self.servo_max_deg = list(self.get_parameter("servo_max_deg").value)

        self.initial_joints_deg = list(
            self.get_parameter("initial_joints_deg").value
        )

        self.send_initial_position = bool(
            self.get_parameter("send_initial_position").value
        )

        self.joint_names = list(self.get_parameter("joint_names").get_parameter_value().string_array_value)
        self.publish_joint_states = (self.get_parameter("publish_joint_states").get_parameter_value().bool_value)
        self.joint_state_rate = (self.get_parameter("joint_state_rate").get_parameter_value().double_value)

        n_joints = len(self.joint_names)
        if not (
            len(self.servo_center_deg) == n_joints and
            len(self.servo_sign) == n_joints and
            len(self.servo_min_deg) == n_joints and
            len(self.servo_max_deg) == n_joints and
            len(self.initial_joints_deg) == n_joints
        ):
            raise RuntimeError("Servo parameter and initial-position arrays must have the same length as joint_names")

        self.ser = serial.Serial(serial_port, baudrate, timeout=1)

        self.subscription = self.create_subscription(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            self.listener_callback,
            10
        )

        self.current_positions = [
            math.radians(value)
            for value in self.initial_joints_deg
        ]

        if self.send_initial_position:
            servo_angles = self.send_joint_positions(
                self.current_positions
            )

            self.get_logger().info(
                "Initial position sent | "
                f"Joint targets [deg]: {self.initial_joints_deg} | "
                f"Servo angles [deg]: {servo_angles}"
            )
        else:
            self.get_logger().info(
                "Initial position not sent automatically"
            )

        if self.publish_joint_states:
            self.joint_state_pub = self.create_publisher(
                JointState,
                "/joint_states",
                10
            )

            self.joint_state_timer = self.create_timer(
                1.0 / self.joint_state_rate,
                self.publish_joint_states_callback
            )

        self.get_logger().info(
            f"Serial trajectory bridge started on {serial_port} at {baudrate} baud"
        )

    def joint_rad_to_servo_deg(self, q_rad, i):
        q_deg = math.degrees(q_rad)

        servo_deg = self.servo_center_deg[i] + self.servo_sign[i] * q_deg

        servo_deg = max(
            self.servo_min_deg[i],
            min(self.servo_max_deg[i], servo_deg)
        )

        return int(round(servo_deg))

    def send_joint_positions(self, joint_positions):
        servo_angles = [
            self.joint_rad_to_servo_deg(
                joint_positions[i], i
            )
            for i in range(len(self.joint_names))
        ]

        data_str = ",".join(
            str(angle) for angle in servo_angles
        ) + "\n"

        self.ser.write(data_str.encode("utf-8"))

        return servo_angles

    def listener_callback(self, msg):
        if len(msg.points) == 0:
            self.get_logger().warn("Received JointTrajectory without points")
            return

        previous_time = 0.0

        self.get_logger().info(
            f"Executing trajectory with {len(msg.points)} points"
        )

        n_joints = len(self.joint_names)
        for point in msg.points:

            if len(point.positions) != n_joints:
                self.get_logger().warn(
                    f"Expected {n_joints} joint positions, received {len(point.positions)}"
                )
                return

            current_time = (
                point.time_from_start.sec +
                point.time_from_start.nanosec * 1e-9
            )

            if current_time < previous_time:
                self.get_logger().warn(
                    "Trajectory time_from_start must be non-decreasing"
                )
                return

            wait_time = current_time - previous_time

            if wait_time > 0.0:
                time.sleep(wait_time)

            servo_angles = self.send_joint_positions(
                point.positions
            )

            self.current_positions = list(point.positions)

            self.get_logger().info(
                f"t={current_time:.2f}s | Servo angles [deg]: {servo_angles}"
            )

            previous_time = current_time

        self.get_logger().info("Trajectory execution finished")

    def publish_joint_states_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = []
        msg.effort = []

        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SerialTrajectoryBridgeNode()

    try:
        rclpy.spin(node)
    finally:
        if hasattr(node, "ser") and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()