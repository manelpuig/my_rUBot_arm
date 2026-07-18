#!/usr/bin/env python3
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
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


class ArmMoveToPoseTaskSpace(Node):
    """Plan to a Cartesian pose goal using the MoveIt MoveGroup action."""

    def __init__(self):
        super().__init__("arm_move_to_pose_task_space")

        self.declare_parameter("startup_delay_sec", 3.0)

        self.declare_parameter("target_xyz", [0.40, 0.00, 0.50])
        self.declare_parameter("target_rpy", [0.0, math.pi, 0.0])

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("planning_frame", "base_link")

        self.declare_parameter("group_name", "arm")
        self.declare_parameter("ik_link", "tool")

        self.declare_parameter("position_tolerance", 0.005)
        self.declare_parameter("orientation_tolerance", 0.01)
        self.declare_parameter("max_velocity", 0.2)
        self.declare_parameter("max_acceleration", 0.2)
        self.declare_parameter("execute", True)

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

        self.position_tolerance = float(
            self.get_parameter("position_tolerance").value
        )
        self.orientation_tolerance = float(
            self.get_parameter("orientation_tolerance").value
        )
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(
            self.get_parameter("max_acceleration").value
        )
        self.execute_motion = bool(self.get_parameter("execute").value)
        self.startup_delay_sec = float(
            self.get_parameter("startup_delay_sec").value
        )

        if len(self.target_xyz) != 3:
            raise ValueError("Parameter 'target_xyz' must contain exactly 3 values.")
        if len(self.target_rpy) != 3:
            raise ValueError("Parameter 'target_rpy' must contain exactly 3 values.")
        if not 0.0 < self.max_velocity <= 1.0:
            raise ValueError("Parameter 'max_velocity' must be in the range (0, 1].")
        if not 0.0 < self.max_acceleration <= 1.0:
            raise ValueError(
                "Parameter 'max_acceleration' must be in the range (0, 1]."
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

    def _run_once(self):
        if self._done:
            return

        pose_target = self._build_target_pose()
        pose_planning = self._transform_pose_to_planning_frame(pose_target)

        if pose_planning is None:
            self.get_logger().warn("TF not available yet. Waiting and retrying...")
            return

        self._done = True

        self.get_logger().info(
            f"Cartesian pose goal in '{self.planning_frame}': "
            f"position=({pose_planning.pose.position.x:.4f}, "
            f"{pose_planning.pose.position.y:.4f}, "
            f"{pose_planning.pose.position.z:.4f}) m"
        )
        self.get_logger().info(
            "Planning to a pose goal. The end-effector path is not constrained "
            "to a Cartesian straight line."
        )

        if self.execute_motion:
            self.get_logger().info("Planning and executing the pose goal...")
            self.moveit2.move_to_pose(
                pose=pose_planning,
                target_link=self.ik_link,
                tolerance_position=self.position_tolerance,
                tolerance_orientation=self.orientation_tolerance,
                cartesian=False,
            )
            self.moveit2.wait_until_executed()

            error_code = self.moveit2.get_last_execution_error_code()
            if error_code is None:
                self.get_logger().info("Motion request finished.")
            else:
                self.get_logger().info(
                    f"Motion request finished with MoveIt error code: {error_code.val}"
                )
        else:
            self.get_logger().info("Planning the pose goal without execution...")
            trajectory = self.moveit2.plan(
                pose=pose_planning,
                target_link=self.ik_link,
                tolerance_position=self.position_tolerance,
                tolerance_orientation=self.orientation_tolerance,
                cartesian=False,
            )

            if trajectory is None or not trajectory.points:
                self.get_logger().error("MoveIt could not find a valid trajectory.")
            else:
                self.get_logger().info(
                    f"Planning succeeded: {len(trajectory.points)} trajectory points."
                )

        self.get_logger().info("Task-space pose node finished.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ArmMoveToPoseTaskSpace()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received. Shutting down...")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
