#!/usr/bin/env python3

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState

import tf2_ros


def euler_from_quaternion_xyzw(qx, qy, qz, qw):
    # Roll around X
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch around Y
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw around Z
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class UR5eFKine(Node):

    def __init__(self):
        super().__init__("ur5e_fkine")

        self.declare_parameter(
            "joints",
            [0.0, -60.0, -135.0, -30.0, 90.0, 0.0],
        )

        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tcp_frame", "tool")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("settling_time", 1.0)

        self.target_deg = list(self.get_parameter("joints").value)
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.base_frame = self.get_parameter("base_frame").value
        self.tcp_frame = self.get_parameter("tcp_frame").value
        self.publish_rate = float(
            self.get_parameter("publish_rate").value
        )
        self.settling_time = float(
            self.get_parameter("settling_time").value
        )

        if len(self.target_deg) != len(self.joint_names):
            raise RuntimeError(
                "target_deg and joint_names must have the same length"
            )

        self.target_rad = [
            math.radians(angle)
            for angle in self.target_deg
        ]

        self.pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
        )

        self.done = False
        self.start_time = self.get_clock().now()

        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_joint_state,
        )

        self.result_timer = self.create_timer(
            0.2,
            self.try_to_print_tcp_pose,
        )

        self.get_logger().info(
            f"Target joints [deg]: {self.target_deg}"
        )
        self.get_logger().info(
            f"Target joints [rad]: {self.target_rad}"
        )
        self.get_logger().info(
            f"Publishing joint states on /joint_states"
        )
        self.get_logger().info(
            f"TF query: {self.base_frame} -> {self.tcp_frame}"
        )

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.target_rad

        self.pub.publish(msg)

    def try_to_print_tcp_pose(self):
        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        if elapsed < self.settling_time:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tcp_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )

            t = tf.transform.translation
            q = tf.transform.rotation

            roll, pitch, yaw = euler_from_quaternion_xyzw(
                q.x, q.y, q.z, q.w
            )

            self.get_logger().info(
                "========== UR5e FKINE =========="
            )
            self.get_logger().info(
                f"Position [m]: "
                f"x={t.x:.4f}, y={t.y:.4f}, z={t.z:.4f}"
            )
            self.get_logger().info(
                "Orientation RPY [deg]: "
                f"roll={math.degrees(roll):.2f}, "
                f"pitch={math.degrees(pitch):.2f}, "
                f"yaw={math.degrees(yaw):.2f}"
            )
            self.get_logger().info(
                "Quaternion: "
                f"x={q.x:.4f}, y={q.y:.4f}, "
                f"z={q.z:.4f}, w={q.w:.4f}"
            )
            self.get_logger().info(
                "================================"
            )

            self.done = True

        except Exception:
            # TF may not be available during the first iterations.
            pass


def main(args=None):
    rclpy.init(args=args)
    node = UR5eFKine()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
