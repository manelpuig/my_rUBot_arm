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


class ArmMoveL(Node):
    """Compute and optionally execute a straight Cartesian tool path."""

    def __init__(self):
        super().__init__("arm_movel")

        self.declare_parameter("startup_delay_sec", 3.0)
        self.declare_parameter("motion_timeout_sec", 180.0)

        self.declare_parameter("target_xyz", [0.0, -0.40, 0.45])
        self.declare_parameter("target_rpy", [math.pi / 2.0, 0.0, 0.0])

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("planning_frame", "base_link")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("ik_link", "tool")

        self.declare_parameter("max_step", 0.005)
        self.declare_parameter("fraction_threshold", 0.95)
        self.declare_parameter("jump_threshold", 0.0)
        self.declare_parameter("avoid_collisions", True)

        self.declare_parameter("max_velocity", 0.2)
        self.declare_parameter("max_acceleration", 0.2)
        self.declare_parameter("execute", False)

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

        self.max_step = float(self.get_parameter("max_step").value)
        self.fraction_threshold = float(
            self.get_parameter("fraction_threshold").value
        )
        self.jump_threshold = float(
            self.get_parameter("jump_threshold").value
        )
        self.avoid_collisions = bool(
            self.get_parameter("avoid_collisions").value
        )

        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(
            self.get_parameter("max_acceleration").value
        )
        self.execute_motion = bool(self.get_parameter("execute").value)
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
        if self.max_step <= 0.0:
            raise ValueError("Parameter 'max_step' must be greater than zero.")
        if not 0.0 <= self.fraction_threshold <= 1.0:
            raise ValueError("Parameter 'fraction_threshold' must be in [0, 1].")
        if self.jump_threshold < 0.0:
            raise ValueError("Parameter 'jump_threshold' cannot be negative.")
        if not 0.0 < self.max_velocity <= 1.0:
            raise ValueError("Parameter 'max_velocity' must be in the range (0, 1].")
        if not 0.0 < self.max_acceleration <= 1.0:
            raise ValueError(
                "Parameter 'max_acceleration' must be in the range (0, 1]."
            )

        self.callback_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        self.moveit2.cartesian_avoid_collisions = self.avoid_collisions
        self.moveit2.cartesian_jump_threshold = self.jump_threshold

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

    def run(self):
        time.sleep(self.startup_delay_sec)

        if not self._wait_for_joint_state():
            return False

        target_pose = self._build_target_pose()
        target_pose = self._transform_pose_to_planning_frame(target_pose)
        if target_pose is None:
            return False

        self.get_logger().info(
            f"MoveL target in '{self.planning_frame}': "
            f"position=({target_pose.pose.position.x:.4f}, "
            f"{target_pose.pose.position.y:.4f}, "
            f"{target_pose.pose.position.z:.4f}) m"
        )
        self.get_logger().info(
            f"Computing Cartesian path: max_step={self.max_step:.4f} m, "
            f"minimum_fraction={self.fraction_threshold:.2f}, "
            f"avoid_collisions={self.avoid_collisions}."
        )

        future = self.moveit2.plan_async(
            pose=target_pose,
            target_link=self.ik_link,
            cartesian=True,
            max_step=self.max_step,
        )

        if future is None:
            self.get_logger().error("The /compute_cartesian_path service is unavailable.")
            return False

        if not self._wait_for_future(future, "Cartesian planning"):
            return False

        response = future.result()
        fraction = float(response.fraction)
        self.get_logger().info(
            f"Cartesian path completed fraction: {fraction:.3f} "
            f"({100.0 * fraction:.1f}%)."
        )

        trajectory = self.moveit2.get_trajectory(
            future,
            cartesian=True,
            cartesian_fraction_threshold=self.fraction_threshold,
        )

        if trajectory is None or not trajectory.points:
            self.get_logger().error(
                "No acceptable Cartesian trajectory was generated."
            )
            return False

        self.get_logger().info(
            f"Cartesian planning succeeded: {len(trajectory.points)} points."
        )

        if not self.execute_motion:
            self.get_logger().info("execute:=false -> trajectory will not be executed.")
            return True

        self.get_logger().info("Executing the Cartesian trajectory...")
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
                f"Cartesian execution failed with MoveIt error code: {error_code.val}"
            )
            return False

        self.get_logger().info("Cartesian trajectory execution succeeded.")
        return True


def main():
    rclpy.init()
    node = ArmMoveL()

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
