#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg

import tf2_ros


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rpy_to_R(roll, pitch, yaw):
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)


def wrap_to_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rad2deg(q):
    return [math.degrees(v) for v in q]


def deg2rad(q):
    return [math.radians(v) for v in q]


def zyz_from_R(R):
    r33 = max(-1.0, min(1.0, float(R[2, 2])))
    q5 = math.acos(r33)

    if abs(math.sin(q5)) < 1e-9:
        q4 = math.atan2(float(R[1, 0]), float(R[0, 0]))
        q6 = 0.0
    else:
        q4 = math.atan2(float(R[1, 2]), float(R[0, 2]))
        q6 = math.atan2(float(R[2, 1]), -float(R[2, 0]))

    return q4, q5, q6


def euler_from_quaternion_xyzw(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class PumaIKinePose(Node):

    def __init__(self):
        super().__init__("puma_ikine_pose")

        self.declare_parameter("target_xyz", [0.5422, 0.1397, 0.9423])
        self.declare_parameter("target_rpy_deg", [0.0, 45.0, 0.0])

        self.declare_parameter("L1", 0.4)
        self.declare_parameter("L2", 0.4318)
        self.declare_parameter("L3", 0.43208)
        self.declare_parameter("d3", 0.1397)

        self.declare_parameter("elbow", "up")
        self.declare_parameter("wrist", "noflip")

        self.declare_parameter("time_sec", 5.0)
        self.declare_parameter("controller_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tcp_frame", "puma_tool")

        self.pub = self.create_publisher(
            JointTrajectory,
            self.get_parameter("controller_topic").value,
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sent = False
        self.done = False
        self.timer = self.create_timer(1.0, self.compute_and_send)

        self.get_logger().info(f"Publishing to: {self.get_parameter('controller_topic').value}")

    def R03(self, q1, q2, q3):
        return Rz(q1) @ Ry(q2 + q3)

    def ikine_pose(self):
        x, y, z = [float(v) for v in self.get_parameter("target_xyz").value]
        roll, pitch, yaw = deg2rad(self.get_parameter("target_rpy_deg").value)

        L1 = float(self.get_parameter("L1").value)
        L2 = float(self.get_parameter("L2").value)
        L3 = float(self.get_parameter("L3").value)
        d3 = float(self.get_parameter("d3").value)

        r = math.sqrt(x * x + y * y)

        if r < abs(d3) + 1e-9:
            raise RuntimeError("Target too close to base axis for given d3.")

        r_xy = math.sqrt(max(0.0, r * r - d3 * d3))
        phi = math.atan2(y, x)
        gamma = math.atan2(d3, r_xy)
        q1 = phi - gamma

        x_p = r_xy
        z_p = z - L1

        R = math.sqrt(x_p * x_p + z_p * z_p)

        if R < 1e-9:
            raise RuntimeError("Degenerate target.")

        C = (x_p * x_p + z_p * z_p + L3 * L3 - L2 * L2) / (2.0 * L3 * R)

        if C < -1.0 or C > 1.0:
            raise RuntimeError("Unreachable target.")

        C = max(-1.0, min(1.0, C))

        gamma2 = math.atan2(x_p, z_p)
        delta = math.acos(C)

        elbow = self.get_parameter("elbow").value.lower()

        if elbow == "up":
            phi_total = gamma2 + delta
        elif elbow == "down":
            phi_total = gamma2 - delta
        else:
            raise RuntimeError("Invalid elbow. Use up/down.")

        cx = (x_p - L3 * math.sin(phi_total)) / L2
        sz = -(z_p - L3 * math.cos(phi_total)) / L2

        q2 = math.atan2(sz, cx)
        q3 = wrap_to_pi(phi_total - q2)

        R06_target = rpy_to_R(roll, pitch, yaw)
        R03 = self.R03(q1, q2, q3)
        R36 = R03.T @ R06_target

        q4, q5, q6 = zyz_from_R(R36)

        wrist = self.get_parameter("wrist").value.lower()

        if wrist == "flip":
            q4 += math.pi
            q5 = -q5
            q6 += math.pi
        elif wrist != "noflip":
            raise RuntimeError("Invalid wrist. Use noflip/flip.")

        q = [wrap_to_pi(v) for v in [q1, q2, q3, q4, q5, q6]]
        return q

    def compute_and_send(self):
        if self.sent:
            return

        try:
            q_rad = self.ikine_pose()
        except RuntimeError as e:
            self.get_logger().error(str(e))
            self.done = True
            return

        msg = JointTrajectory()
        msg.joint_names = list(self.get_parameter("joint_names").value)

        point = JointTrajectoryPoint()
        point.positions = q_rad

        time_sec = float(self.get_parameter("time_sec").value)
        secs = int(time_sec)
        nsecs = int((time_sec - secs) * 1e9)
        point.time_from_start = DurationMsg(sec=secs, nanosec=nsecs)

        msg.points.append(point)
        self.pub.publish(msg)

        self.get_logger().info(f"Target XYZ [m]: {self.get_parameter('target_xyz').value}")
        self.get_logger().info(f"Target RPY [deg]: {self.get_parameter('target_rpy_deg').value}")
        self.get_logger().info(f"IK solution [deg]: {rad2deg(q_rad)}")
        self.get_logger().info(f"IK solution [rad]: {q_rad}")

        self.sent = True
        self.timer.cancel()

        self.finish_timer = self.create_timer(time_sec + 0.5, self.verify_and_finish)

    def verify_and_finish(self):
        base = self.get_parameter("base_frame").value
        tcp = self.get_parameter("tcp_frame").value

        try:
            tf = self.tf_buffer.lookup_transform(
                base,
                tcp,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0)
            )

            t = tf.transform.translation
            q = tf.transform.rotation

            roll, pitch, yaw = euler_from_quaternion_xyzw(q.x, q.y, q.z, q.w)

            self.get_logger().info("========== PUMA IKINE POSE VERIFY ==========")
            self.get_logger().info(f"Pose {base} -> {tcp}")
            self.get_logger().info(f"Position [m]: x={t.x:.4f}, y={t.y:.4f}, z={t.z:.4f}")
            self.get_logger().info(
                "RPY [deg]: "
                f"roll={math.degrees(roll):.2f}, "
                f"pitch={math.degrees(pitch):.2f}, "
                f"yaw={math.degrees(yaw):.2f}"
            )
            self.get_logger().info("===========================================")

        except Exception as e:
            self.get_logger().warn(f"TF verification failed: {e}")

        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = PumaIKinePose()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()