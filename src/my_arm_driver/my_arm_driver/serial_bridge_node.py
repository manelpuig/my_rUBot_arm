#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import serial


class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__("serial_bridge_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("servo_center_deg", [90, 90, 90, 90, 90, 90])
        self.declare_parameter("servo_sign", [1, 1, 1, 1, 1, 1])

        serial_port = self.get_parameter("serial_port").value
        baudrate = int(self.get_parameter("baudrate").value)

        self.servo_center_deg = list(self.get_parameter("servo_center_deg").value)
        self.servo_sign = list(self.get_parameter("servo_sign").value)

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

        # ROS joint angle 0 rad -> servo neutral position
        servo_deg = self.servo_center_deg[i] + self.servo_sign[i] * q_deg

        return int(max(0, min(180, round(servo_deg))))

    def listener_callback(self, msg):
        if len(msg.points) == 0:
            self.get_logger().warn("Received JointTrajectory without points")
            return

        point = msg.points[-1]

        if len(point.positions) != 6:
            self.get_logger().warn(
                f"Expected 6 joint positions, received {len(point.positions)}"
            )
            return

        servo_angles = [
            self.joint_rad_to_servo_deg(point.positions[i], i)
            for i in range(6)
        ]

        data_str = ",".join(str(a) for a in servo_angles) + "\n"
        self.ser.write(data_str.encode("utf-8"))

        self.get_logger().info(
            f"ROS joints [rad]: {[round(q, 3) for q in point.positions]}"
        )
        self.get_logger().info(
            f"Servo angles [deg]: {servo_angles}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()