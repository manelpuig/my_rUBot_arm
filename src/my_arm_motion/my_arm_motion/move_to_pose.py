#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK

from pymoveit2 import MoveIt2


PUMA_JOINTS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]


def quat_from_rpy(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_angle_near_reference(angle, reference):
    return reference + math.atan2(
        math.sin(angle - reference),
        math.cos(angle - reference),
    )


class PumaMoveToPoseViaIK(Node):

    def __init__(self):
        super().__init__("puma_move_to_pose_via_ik")

        self.declare_parameter("startup_delay_sec", 3.0)

        self.declare_parameter("target_xyz", [0.4, 0.0, 0.3])
        self.declare_parameter("target_rpy", [0.0, math.pi, 0.0])

        self.declare_parameter("planning_frame", "base_link")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("ik_link", "puma_tool")

        self.declare_parameter(
            "seed_joints",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

        self.declare_parameter("ik_timeout_sec", 0.5)
        self.declare_parameter("max_velocity", 0.2)
        self.declare_parameter("max_acceleration", 0.2)
        self.declare_parameter("execute", True)
        self.declare_parameter("print_joints", True)
        self.declare_parameter("seed_from_joint_states", True)

        self.target_xyz = [float(x) for x in self.get_parameter("target_xyz").value]
        self.target_rpy = [float(x) for x in self.get_parameter("target_rpy").value]

        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.ik_link = str(self.get_parameter("ik_link").value)

        self.seed_joints = [float(x) for x in self.get_parameter("seed_joints").value]
        self.ik_timeout = float(self.get_parameter("ik_timeout_sec").value)

        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.execute_motion = bool(self.get_parameter("execute").value)
        self.print_joints = bool(self.get_parameter("print_joints").value)
        self.seed_from_joint_states = bool(self.get_parameter("seed_from_joint_states").value)
        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)

        self._last_js = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
        )

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=PUMA_JOINTS,
            base_link_name=self.planning_frame,
            end_effector_name=self.ik_link,
            group_name=self.group_name,
            use_move_group_action=False,
        )

        self.moveit2.max_velocity = self.max_velocity
        self.moveit2.max_acceleration = self.max_acceleration

        self._done = False
        self.create_timer(self.startup_delay_sec, self._run_once)

    def _joint_state_callback(self, msg):
        self._last_js = msg

    def _build_target_pose(self):
        qx, qy, qz, qw = quat_from_rpy(
            self.target_rpy[0],
            self.target_rpy[1],
            self.target_rpy[2],
        )

        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = self.target_xyz[0]
        pose.pose.position.y = self.target_xyz[1]
        pose.pose.position.z = self.target_xyz[2]

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def _run_once(self):
        if self._done:
            return

        if not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /compute_ik service...")
            return

        if self.seed_from_joint_states and self._last_js is None:
            self.get_logger().warn("Waiting for /joint_states...")
            return

        self._done = True

        pose_target = self._build_target_pose()

        self.get_logger().info(
            f"Pose goal in {self.planning_frame}: "
            f"xyz={self.target_xyz}, rpy={self.target_rpy}"
        )

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link
        req.ik_request.pose_stamped = pose_target

        req.ik_request.timeout.sec = int(self.ik_timeout)
        req.ik_request.timeout.nanosec = int(
            (self.ik_timeout - int(self.ik_timeout)) * 1e9
        )

        seed_positions = list(self.seed_joints)

        if self.seed_from_joint_states and self._last_js is not None:
            name_to_pos = dict(zip(self._last_js.name, self._last_js.position))
            if all(j in name_to_pos for j in PUMA_JOINTS):
                seed_positions = [float(name_to_pos[j]) for j in PUMA_JOINTS]
                self.get_logger().info("Using /joint_states as IK seed.")
            else:
                self.get_logger().warn(
                    "/joint_states does not contain all PUMA joints. "
                    "Using seed_joints."
                )

        seed = JointState()
        seed.header = pose_target.header
        seed.name = PUMA_JOINTS
        seed.position = seed_positions

        req.ik_request.robot_state.joint_state = seed

        future = self.ik_client.call_async(req)
        future.add_done_callback(self._on_ik_response)

    def _normalize_joint_goal(self, joint_goal):
        if self._last_js is None:
            return [wrap_to_pi(q) for q in joint_goal]

        name_to_pos = dict(zip(self._last_js.name, self._last_js.position))

        if not all(j in name_to_pos for j in PUMA_JOINTS):
            return [wrap_to_pi(q) for q in joint_goal]

        current_joints = [float(name_to_pos[j]) for j in PUMA_JOINTS]

        normalized_goal = [
            normalize_angle_near_reference(goal, current)
            for goal, current in zip(joint_goal, current_joints)
        ]

        return [wrap_to_pi(q) for q in normalized_goal]

    def _on_ik_response(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")
            rclpy.shutdown()
            return

        if res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error(f"IK failed. Error code: {res.error_code.val}")
            rclpy.shutdown()
            return

        sol = res.solution.joint_state
        name_to_pos = dict(zip(sol.name, sol.position))

        try:
            joint_goal = [float(name_to_pos[j]) for j in PUMA_JOINTS]
        except KeyError as e:
            self.get_logger().error(f"IK solution missing joint: {e}")
            rclpy.shutdown()
            return

        joint_goal = self._normalize_joint_goal(joint_goal)

        if self.print_joints:
            self.get_logger().info("IK joint goal:")
            for name, value in zip(PUMA_JOINTS, joint_goal):
                self.get_logger().info(f"  {name}: {value:.4f} rad")

        if not self.execute_motion:
            self.get_logger().info("execute:=false -> not moving.")
            rclpy.shutdown()
            return

        self.get_logger().info("Executing joint goal with move_to_configuration()...")
        self.moveit2.move_to_configuration(joint_goal)
        self.moveit2.wait_until_executed()
        self.get_logger().info("Execution finished.")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = PumaMoveToPoseViaIK()
    rclpy.spin(node)


if __name__ == "__main__":
    main()