#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from pymoveit2 import MoveIt2


ARM_JOINTS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]


def quat_from_rpy_zyx(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return float(qx), float(qy), float(qz), float(qw)


class ArmMoveToPoseViaIK(Node):

    def __init__(self):
        super().__init__("arm_move_to_pose")

        self.declare_parameter("startup_delay_sec", 3.0)

        self.declare_parameter("target_xyz", [0.40, 0.00, 0.50])
        self.declare_parameter("target_rpy", [0.0, math.pi, 0.0])

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("planning_frame", "base_link")

        self.declare_parameter("group_name", "arm")
        self.declare_parameter("ik_link", "tool")

        self.declare_parameter(
            "seed_joints",
            [0.0, -0.7, 1.2, 0.0, 0.7, 0.0],
        )
        self.declare_parameter("seed_from_joint_states", True)
        self.declare_parameter("ik_timeout_sec", 0.2)

        self.declare_parameter("max_velocity", 0.3)
        self.declare_parameter("max_acceleration", 0.3)
        self.declare_parameter("execute", True)
        self.declare_parameter("print_joints", True)

        self.target_xyz = [float(x) for x in self.get_parameter("target_xyz").value]
        self.target_rpy = [float(x) for x in self.get_parameter("target_rpy").value]

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.planning_frame = str(self.get_parameter("planning_frame").value)

        self.group_name = str(self.get_parameter("group_name").value)
        self.ik_link = str(self.get_parameter("ik_link").value)

        self.seed_joints = [float(x) for x in self.get_parameter("seed_joints").value]
        self.seed_from_joint_states = bool(
            self.get_parameter("seed_from_joint_states").value
        )
        self.ik_timeout = float(self.get_parameter("ik_timeout_sec").value)

        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.execute_motion = bool(self.get_parameter("execute").value)
        self.print_joints = bool(self.get_parameter("print_joints").value)
        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)

        if len(self.target_xyz) != 3:
            raise ValueError("Parameter 'target_xyz' must contain exactly 3 values.")
        if len(self.target_rpy) != 3:
            raise ValueError("Parameter 'target_rpy' must contain exactly 3 values.")
        if len(self.seed_joints) != 6:
            raise ValueError("Parameter 'seed_joints' must contain exactly 6 values.")

        self._last_js = None
        self.create_subscription(
            JointState,
            "/joint_states",
            self._js_cb,
            qos_profile_sensor_data,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ARM_JOINTS,
            base_link_name=self.planning_frame,
            end_effector_name=self.ik_link,
            group_name=self.group_name,
        )
        self.moveit2.max_velocity = self.max_velocity
        self.moveit2.max_acceleration = self.max_acceleration

        self._done = False
        self.create_timer(self.startup_delay_sec, self._run_once)

    def _js_cb(self, msg: JointState):
        self._last_js = msg

    def _build_target_pose(self):
        qx, qy, qz, qw = quat_from_rpy_zyx(*self.target_rpy)

        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = self.target_xyz[0]
        pose.pose.position.y = self.target_xyz[1]
        pose.pose.position.z = self.target_xyz[2]

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def _transform_pose_to_planning_frame(self, pose_in):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame,
                pose_in.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0),
            )

            pose_out = PoseStamped()
            pose_out.header.frame_id = self.planning_frame
            pose_out.header.stamp = self.get_clock().now().to_msg()
            pose_out.pose = tf2_geometry_msgs.do_transform_pose(
                pose_in.pose,
                transform,
            )

            return pose_out

        except Exception as e:
            self.get_logger().error(
                f"Could not transform pose from '{pose_in.header.frame_id}' "
                f"to '{self.planning_frame}': {e}"
            )
            return None

    def _run_once(self):
        if self._done:
            return

        if not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /compute_ik service...")
            return

        if self.seed_from_joint_states and self._last_js is None:
            self.get_logger().warn("Waiting for /joint_states...")
            return

        pose_target = self._build_target_pose()

        self.get_logger().info(
            f"Pose goal input ({self.target_frame} frame): "
            f"xyz={self.target_xyz}, rpy={self.target_rpy}"
        )

        pose_base = self._transform_pose_to_planning_frame(pose_target)

        if pose_base is None:
            self.get_logger().warn("TF not available yet. Waiting and retrying...")
            return

        self._done = True

        self.get_logger().info(
            f"Pose goal transformed to {self.planning_frame}: "
            f"x={pose_base.pose.position.x:.4f}, "
            f"y={pose_base.pose.position.y:.4f}, "
            f"z={pose_base.pose.position.z:.4f}"
        )

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link
        req.ik_request.pose_stamped = pose_base

        req.ik_request.timeout.sec = int(self.ik_timeout)
        req.ik_request.timeout.nanosec = int(
            (self.ik_timeout - int(self.ik_timeout)) * 1e9
        )

        seed_positions = list(self.seed_joints)

        if self.seed_from_joint_states and self._last_js is not None:
            name_to_pos = dict(zip(self._last_js.name, self._last_js.position))
            if all(j in name_to_pos for j in ARM_JOINTS):
                seed_positions = [float(name_to_pos[j]) for j in ARM_JOINTS]
                self.get_logger().info("Using /joint_states as IK seed.")
            else:
                self.get_logger().warn(
                    "/joint_states received but does not contain all ARM joints. "
                    "Falling back to seed_joints."
                )
        else:
            self.get_logger().warn(
                "No /joint_states received yet. Falling back to seed_joints."
            )

        seed = JointState()
        seed.header = pose_base.header
        seed.name = ARM_JOINTS
        seed.position = seed_positions

        req.ik_request.robot_state.joint_state = seed

        future = self.ik_client.call_async(req)
        future.add_done_callback(self._on_ik)

    def _on_ik(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")
            rclpy.shutdown()
            return

        if res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error(f"IK failed, error code: {res.error_code.val}")
            rclpy.shutdown()
            return

        sol = res.solution.joint_state
        name_to_pos = dict(zip(sol.name, sol.position))

        try:
            joint_goal = [float(name_to_pos[j]) for j in ARM_JOINTS]
        except KeyError as e:
            self.get_logger().error(f"IK solution missing expected joint: {e}")
            rclpy.shutdown()
            return

        if self.print_joints:
            self.get_logger().info("IK joint goal:")
            for n, v in zip(ARM_JOINTS, joint_goal):
                self.get_logger().info(f"  {n}: {v:.4f} rad")

        if not self.execute_motion:
            self.get_logger().info("execute:=false -> exiting without motion.")
            rclpy.shutdown()
            return

        self.get_logger().info("Executing IK joint goal via MoveIt2...")
        self.moveit2.move_to_configuration(joint_goal)
        self.moveit2.wait_until_executed()
        self.get_logger().info("Execution finished.")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = ArmMoveToPoseViaIK()
    rclpy.spin(node)


if __name__ == "__main__":
    main()