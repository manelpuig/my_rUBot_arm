#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
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

        serial_port = self.get_parameter("serial_port").value
        baudrate = int(self.get_parameter("baudrate").value)

        self.servo_center_deg = list(self.get_parameter("servo_center_deg").value)
        self.servo_sign = list(self.get_parameter("servo_sign").value)
        self.servo_min_deg = list(self.get_parameter("servo_min_deg").value)
        self.servo_max_deg = list(self.get_parameter("servo_max_deg").value)

        self.ser = serial.Serial(serial_port, baudrate, timeout=1)

        self.subscription = self.create_subscription(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            self.listener_callback,
            10
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

    def listener_callback(self, msg):
        if len(msg.points) == 0:
            self.get_logger().warn("Received JointTrajectory without points")
            return

        previous_time = 0.0

        self.get_logger().info(
            f"Executing trajectory with {len(msg.points)} points"
        )

        for point in msg.points:

            if len(point.positions) != 6:
                self.get_logger().warn(
                    f"Expected 6 joint positions, received {len(point.positions)}"
                )
                return

            current_time = (
                point.time_from_start.sec +
                point.time_from_start.nanosec * 1e-9
            )

            wait_time = current_time - previous_time

            if wait_time > 0.0:
                time.sleep(wait_time)

            servo_angles = [
                self.joint_rad_to_servo_deg(point.positions[i], i)
                for i in range(6)
            ]

            data_str = ",".join(str(a) for a in servo_angles) + "\n"
            self.ser.write(data_str.encode("utf-8"))

            self.get_logger().info(
                f"t={current_time:.2f}s | Servo angles [deg]: {servo_angles}"
            )

            previous_time = current_time

        self.get_logger().info("Trajectory execution finished")


def main(args=None):
    rclpy.init(args=args)
    node = SerialTrajectoryBridgeNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()