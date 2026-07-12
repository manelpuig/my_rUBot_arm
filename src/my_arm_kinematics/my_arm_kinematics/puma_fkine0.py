#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg

import tf2_ros

def euler_from_quaternion_xyzw(qx, qy, qz, qw):
    # roll X
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch Y
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw Z
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class PumaFKine(Node):

    def __init__(self):
        super().__init__("puma_fkine")

        self.declare_parameter(
            "target_deg",
            [0.0, -30.0, 60.0, 0.0, 45.0, 0.0]
        )

        self.declare_parameter("time_sec", 5.0)

        self.declare_parameter(
            "controller_topic",
            "/arm_controller/joint_trajectory"
        )

        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        )

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tcp_frame", "puma_tool")

        self.target_deg = list(self.get_parameter("target_deg").value)
        self.time_sec = float(self.get_parameter("time_sec").value)
        self.controller_topic = self.get_parameter("controller_topic").value
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.base_frame = self.get_parameter("base_frame").value
        self.tcp_frame = self.get_parameter("tcp_frame").value

        if len(self.target_deg) != len(self.joint_names):
            raise RuntimeError(
                "target_deg and joint_names must have the same length"
            )

        self.pub = self.create_publisher(
            JointTrajectory,
            self.controller_topic,
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sent = False
        self.done = False

        self.timer = self.create_timer(1.0, self.send_trajectory)

        self.get_logger().info(f"Publishing to: {self.controller_topic}")
        self.get_logger().info(f"Base frame: {self.base_frame}")
        self.get_logger().info(f"TCP frame : {self.tcp_frame}")

    def send_trajectory(self):
        if self.sent:
            return

        target_rad = [math.radians(q) for q in self.target_deg]

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = target_rad

        secs = int(self.time_sec)
        nsecs = int((self.time_sec - secs) * 1e9)
        point.time_from_start = DurationMsg(sec=secs, nanosec=nsecs)

        msg.points.append(point)

        self.pub.publish(msg)

        self.get_logger().info(f"Target joints [deg]: {self.target_deg}")
        self.get_logger().info(f"Target joints [rad]: {target_rad}")

        self.sent = True
        self.timer.cancel()

        self.finish_timer = self.create_timer(
            self.time_sec + 0.5,
            self.print_tcp_pose_and_finish
        )

    def print_tcp_pose_and_finish(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tcp_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0)
            )

            t = tf.transform.translation
            q = tf.transform.rotation

            roll, pitch, yaw = euler_from_quaternion_xyzw(
                q.x, q.y, q.z, q.w
            )

            self.get_logger().info("========== PUMA FKINE ==========")
            self.get_logger().info(
                f"Position [m]: x={t.x:.4f}, y={t.y:.4f}, z={t.z:.4f}"
            )
            self.get_logger().info(
                "Orientation RPY [deg]: "
                f"roll={math.degrees(roll):.2f}, "
                f"pitch={math.degrees(pitch):.2f}, "
                f"yaw={math.degrees(yaw):.2f}"
            )
            self.get_logger().info(
                "Quaternion: "
                f"x={q.x:.4f}, y={q.y:.4f}, z={q.z:.4f}, w={q.w:.4f}"
            )
            self.get_logger().info("===============================")

        except Exception as e:
            self.get_logger().error(
                f"Could not read TF {self.base_frame} -> {self.tcp_frame}: {e}"
            )

        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = PumaFKine()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()