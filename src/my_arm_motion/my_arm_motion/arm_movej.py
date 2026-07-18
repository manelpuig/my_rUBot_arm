#!/usr/bin/env python3
import math
import time
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from pymoveit2 import MoveIt2, MoveIt2State


ARM_JOINTS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]


def quat_from_rpy_zyx(roll: float, pitch: float, yaw: float):
    """Convert roll, pitch and yaw in radians to an XYZW quaternion."""
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


class ArmMoveJ(Node):
    """Plan and optionally execute a MoveJ motion to a Cartesian target."""

    def __init__(self):
        super().__init__("arm_movej")

        self.declare_parameter("startup_delay_sec", 3.0)
        self.declare_parameter("motion_timeout_sec", 180.0)

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
        self.declare_parameter("ik_timeout_sec", 0.5)
        self.declare_parameter("avoid_collisions", True)

        self.declare_parameter("joint_tolerance", 0.001)
        self.declare_parameter("max_velocity", 0.2)
        self.declare_parameter("max_acceleration", 0.2)
        self.declare_parameter("execute", False)
        self.declare_parameter("print_joints", True)

        self.target_xyz = [
            float(x) for x in self.get_parameter("target_xyz").value
        ]
        self.target_rpy = [
            float(x) for x in self.get_parameter("target_rpy").value
        ]

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.ik_link = str(self.get_parameter("ik_link").value)

        self.seed_joints = [
            float(x) for x in self.get_parameter("seed_joints").value
        ]
        self.seed_from_joint_states = bool(
            self.get_parameter("seed_from_joint_states").value
        )
        self.ik_timeout_sec = float(
            self.get_parameter("ik_timeout_sec").value
        )
        self.avoid_collisions = bool(
            self.get_parameter("avoid_collisions").value
        )

        self.joint_tolerance = float(
            self.get_parameter("joint_tolerance").value
        )
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(
            self.get_parameter("max_acceleration").value
        )
        self.execute_motion = bool(self.get_parameter("execute").value)
        self.print_joints = bool(self.get_parameter("print_joints").value)
        self.startup_delay_sec = float(
            self.get_parameter("startup_delay_sec").value
        )
        self.motion_timeout_sec = float(
            self.get_parameter("motion_timeout_sec").value
        )

        if len(self.target_xyz) != 3:
            raise ValueError("Parameter 'target_xyz' must contain exactly 3 values.")
        if len(self.target_rpy) != 3:
            raise ValueError("Parameter 'target_rpy' must contain exactly 3 values.")
        if len(self.seed_joints) != len(ARM_JOINTS):
            raise ValueError("Parameter 'seed_joints' must contain exactly 6 values.")
        if self.ik_timeout_sec <= 0.0:
            raise ValueError("Parameter 'ik_timeout_sec' must be greater than zero.")
        if self.joint_tolerance <= 0.0:
            raise ValueError("Parameter 'joint_tolerance' must be greater than zero.")
        if not 0.0 < self.max_velocity <= 1.0:
            raise ValueError("Parameter 'max_velocity' must be in the range (0, 1].")
        if not 0.0 < self.max_acceleration <= 1.0:
            raise ValueError(
                "Parameter 'max_acceleration' must be in the range (0, 1]."
            )

        self.callback_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ik_client = self.create_client(
            GetPositionIK,
            "/compute_ik",
            callback_group=self.callback_group,
        )

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ARM_JOINTS,
            base_link_name=self.planning_frame,
            end_effector_name=self.ik_link,
            group_name=self.group_name,
            callback_group=self.callback_group,
        )
        self.moveit2.max_velocity = self.max_velocity
        self.moveit2.max_acceleration = self.max_acceleration

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
        if pose_in.header.frame_id == self.planning_frame:
            return pose_in

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

        except Exception as error:
            self.get_logger().error(
                f"Could not transform pose from '{pose_in.header.frame_id}' "
                f"to '{self.planning_frame}': {error}"
            )
            return None

    def _wait_for_joint_state(self):
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and self.moveit2.joint_state is None:
            if time.monotonic() >= deadline:
                self.get_logger().error(
                    "No /joint_states message was received after 10 seconds."
                )
                return False
            self.get_logger().info("Waiting for /joint_states...")
            time.sleep(0.2)

        self.get_logger().info("Current joint state is available.")
        return True

    def _wait_for_future(self, future, operation_name):
        deadline = time.monotonic() + self.motion_timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                self.get_logger().error(
                    f"{operation_name} did not finish before the timeout."
                )
                return False
            time.sleep(0.1)
        return future.done()

    def _wait_for_execution(self):
        deadline = time.monotonic() + self.motion_timeout_sec
        while rclpy.ok():
            if self.moveit2.query_state() == MoveIt2State.IDLE:
                return True
            if time.monotonic() >= deadline:
                self.get_logger().error(
                    "Trajectory execution did not finish before the timeout."
                )
                return False
            time.sleep(0.1)
        return False

    def _get_seed_positions(self, current_state):
        if not self.seed_from_joint_states:
            self.get_logger().info("Using manual seed_joints as IK seed.")
            return list(self.seed_joints)

        name_to_position = dict(
            zip(current_state.name, current_state.position)
        )
        if all(joint in name_to_position for joint in ARM_JOINTS):
            self.get_logger().info("Using /joint_states as IK seed.")
            return [
                float(name_to_position[joint]) for joint in ARM_JOINTS
            ]

        self.get_logger().warn(
            "/joint_states does not contain all arm joints. "
            "Using manual seed_joints instead."
        )
        return list(self.seed_joints)

    def _build_ik_request(self, target_pose, seed_positions):
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.ik_link
        request.ik_request.pose_stamped = target_pose
        request.ik_request.avoid_collisions = self.avoid_collisions

        request.ik_request.timeout.sec = int(self.ik_timeout_sec)
        request.ik_request.timeout.nanosec = int(
            (self.ik_timeout_sec - int(self.ik_timeout_sec)) * 1e9
        )

        seed = JointState()
        seed.header = target_pose.header
        seed.name = list(ARM_JOINTS)
        seed.position = list(seed_positions)
        request.ik_request.robot_state.joint_state = seed

        return request

    def _extract_joint_goal(self, response):
        if response.error_code.val != response.error_code.SUCCESS:
            self.get_logger().error(
                f"IK failed with MoveIt error code: {response.error_code.val}"
            )
            return None

        name_to_position = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        )

        try:
            return [float(name_to_position[joint]) for joint in ARM_JOINTS]
        except KeyError as error:
            self.get_logger().error(
                f"IK solution does not contain the expected joint: {error}"
            )
            return None

    def run(self):
        time.sleep(self.startup_delay_sec)

        if not self._wait_for_joint_state():
            return False

        current_state = self.moveit2.joint_state
        target_pose = self._build_target_pose()
        target_pose = self._transform_pose_to_planning_frame(target_pose)
        if target_pose is None:
            return False

        self.get_logger().info(
            f"MoveJ target in '{self.planning_frame}': "
            f"position=({target_pose.pose.position.x:.4f}, "
            f"{target_pose.pose.position.y:.4f}, "
            f"{target_pose.pose.position.z:.4f}) m"
        )

        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("The /compute_ik service is unavailable.")
            return False

        seed_positions = self._get_seed_positions(current_state)
        ik_request = self._build_ik_request(target_pose, seed_positions)

        self.get_logger().info(
            f"Computing collision-aware IK: avoid_collisions={self.avoid_collisions}."
        )
        ik_future = self.ik_client.call_async(ik_request)
        if not self._wait_for_future(ik_future, "IK request"):
            return False

        joint_goal = self._extract_joint_goal(ik_future.result())
        if joint_goal is None:
            return False

        if self.print_joints:
            self.get_logger().info("MoveJ joint goal:")
            for joint_name, value in zip(ARM_JOINTS, joint_goal):
                self.get_logger().info(
                    f"  {joint_name}: {math.degrees(value):.4f} deg "
                    f"({value:.4f} rad)"
                )

        self.get_logger().info("Planning collision-aware MoveJ trajectory with OMPL...")
        planning_future = self.moveit2.plan_async(
            joint_positions=joint_goal,
            joint_names=ARM_JOINTS,
            tolerance_joint_position=self.joint_tolerance,
            start_joint_state=current_state,
            cartesian=False,
        )

        if planning_future is None:
            self.get_logger().error("The MoveIt planning service is unavailable.")
            return False
        if not self._wait_for_future(planning_future, "MoveJ planning"):
            return False

        trajectory = self.moveit2.get_trajectory(
            planning_future,
            cartesian=False,
        )
        if trajectory is None or not trajectory.points:
            self.get_logger().error("MoveIt could not generate a MoveJ trajectory.")
            return False

        self.get_logger().info(
            f"MoveJ planning succeeded: {len(trajectory.points)} trajectory points."
        )

        if not self.execute_motion:
            self.get_logger().info("execute:=false -> trajectory will not be executed.")
            return True

        self.get_logger().info("Executing the MoveJ trajectory...")
        self.moveit2.execute(trajectory)

        if not self._wait_for_execution():
            return False

        error_code = self.moveit2.get_last_execution_error_code()
        if error_code is None:
            self.get_logger().error(
                "Execution finished without a MoveIt result code. "
                "Check that /execute_trajectory is available."
            )
            return False
        if error_code.val != error_code.SUCCESS:
            self.get_logger().error(
                f"MoveJ execution failed with MoveIt error code: {error_code.val}"
            )
            return False

        self.get_logger().info("MoveJ trajectory execution succeeded.")
        return True


def main():
    rclpy.init()
    node = ArmMoveJ()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received. Shutting down...")
    finally:
        executor.shutdown(timeout_sec=2.0)
        executor_thread.join(timeout=2.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
