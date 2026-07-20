#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


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


class ArmPoseNumericIK(Node):
    """Compute numerical IK and optionally send one direct joint target."""

    def __init__(self):
        super().__init__("arm_pose_numeric_ik")

        self.declare_parameter("target_xyz", [0.0, -0.4, 0.5])
        self.declare_parameter("target_rpy", [math.pi / 2.0, 0.0, 0.0])

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("planning_frame", "base_link")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("ik_link", "tool")

        self.declare_parameter("seed_from_joint_states", True)
        self.declare_parameter(
            "seed_joints",
            [
                math.radians(-60.0),
                math.radians(-60.0),
                math.radians(-100.0),
                math.radians(170.0),
                math.radians(-90.0),
                0.0,
            ],
        )
        self.declare_parameter("ik_timeout_sec", 1.0)

        self.declare_parameter(
            "controller_action",
            "/arm_controller/follow_joint_trajectory",
        )
        self.declare_parameter("duration_sec", 4.0)
        self.declare_parameter("execute", False)
        self.declare_parameter("print_joints", True)

        self.target_xyz = [
            float(value) for value in self.get_parameter("target_xyz").value
        ]
        self.target_rpy = [
            float(value) for value in self.get_parameter("target_rpy").value
        ]
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.planning_frame = str(
            self.get_parameter("planning_frame").value
        )
        self.group_name = str(self.get_parameter("group_name").value)
        self.ik_link = str(self.get_parameter("ik_link").value)

        self.seed_from_joint_states = bool(
            self.get_parameter("seed_from_joint_states").value
        )
        self.seed_joints = [
            float(value) for value in self.get_parameter("seed_joints").value
        ]
        self.ik_timeout_sec = float(
            self.get_parameter("ik_timeout_sec").value
        )

        self.controller_action = str(
            self.get_parameter("controller_action").value
        )
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.execute_motion = bool(self.get_parameter("execute").value)
        self.print_joints = bool(self.get_parameter("print_joints").value)

        if len(self.target_xyz) != 3:
            raise ValueError("'target_xyz' must contain exactly three values.")
        if len(self.target_rpy) != 3:
            raise ValueError("'target_rpy' must contain exactly three values.")
        if len(self.seed_joints) != len(ARM_JOINTS):
            raise ValueError("'seed_joints' must contain exactly six values.")
        if self.ik_timeout_sec <= 0.0:
            raise ValueError("'ik_timeout_sec' must be greater than zero.")
        if self.duration_sec <= 0.0:
            raise ValueError("'duration_sec' must be greater than zero.")

        self.current_joint_state = None
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.controller_action,
        )

    def _joint_state_callback(self, message):
        self.current_joint_state = message

    def _wait_until(self, condition, timeout_sec, operation):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if condition():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error(f"Timeout while waiting for {operation}.")
        return False

    def _wait_for_future(self, future, timeout_sec, operation):
        return self._wait_until(future.done, timeout_sec, operation)

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

    def _transform_to_planning_frame(self, pose):
        if pose.header.frame_id == self.planning_frame:
            return pose

        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame,
                pose.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0),
            )
            transformed = PoseStamped()
            transformed.header.frame_id = self.planning_frame
            transformed.header.stamp = self.get_clock().now().to_msg()
            transformed.pose = tf2_geometry_msgs.do_transform_pose(
                pose.pose,
                transform,
            )
            return transformed
        except Exception as error:
            self.get_logger().error(
                f"Could not transform pose from '{pose.header.frame_id}' "
                f"to '{self.planning_frame}': {error}"
            )
            return None

    def _seed_positions(self):
        if not self.seed_from_joint_states:
            self.get_logger().info("Using manual seed_joints as IK seed.")
            return list(self.seed_joints)

        if self.current_joint_state is None:
            self.get_logger().warn(
                "No current joint state is available. Using seed_joints."
            )
            return list(self.seed_joints)

        positions = dict(
            zip(
                self.current_joint_state.name,
                self.current_joint_state.position,
            )
        )
        if not all(joint in positions for joint in ARM_JOINTS):
            self.get_logger().warn(
                "/joint_states does not contain all arm joints. "
                "Using seed_joints."
            )
            return list(self.seed_joints)

        self.get_logger().info("Using /joint_states as IK seed.")
        return [float(positions[joint]) for joint in ARM_JOINTS]

    def _build_ik_request(self, target_pose, seed_positions):
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.ik_link
        request.ik_request.pose_stamped = target_pose
        request.ik_request.avoid_collisions = False

        request.ik_request.timeout.sec = int(self.ik_timeout_sec)
        request.ik_request.timeout.nanosec = int(
            (self.ik_timeout_sec - int(self.ik_timeout_sec)) * 1.0e9
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

        positions = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        )
        try:
            return [float(positions[joint]) for joint in ARM_JOINTS]
        except KeyError as error:
            self.get_logger().error(
                f"IK solution does not contain the expected joint: {error}"
            )
            return None

    def _execute_joint_goal(self, joint_goal):
        if not self.trajectory_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                f"Action server '{self.controller_action}' is unavailable."
            )
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()

        point = JointTrajectoryPoint()
        point.positions = list(joint_goal)
        point.velocities = [0.0] * len(ARM_JOINTS)
        point.time_from_start.sec = int(self.duration_sec)
        point.time_from_start.nanosec = int(
            (self.duration_sec - int(self.duration_sec)) * 1.0e9
        )
        goal.trajectory.points = [point]

        self.get_logger().info(
            f"Sending direct joint target; duration={self.duration_sec:.2f} s."
        )
        send_future = self.trajectory_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, 5.0, "goal acceptance"):
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(
            result_future,
            self.duration_sec + 10.0,
            "trajectory execution",
        ):
            return False

        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"Trajectory execution failed with code {result.error_code}."
            )
            return False

        self.get_logger().info("Direct joint target executed successfully.")
        return True

    def run(self):
        if self.seed_from_joint_states:
            self._wait_until(
                lambda: self.current_joint_state is not None,
                5.0,
                "/joint_states",
            )

        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("The /compute_ik service is unavailable.")
            return False

        target_pose = self._transform_to_planning_frame(
            self._build_target_pose()
        )
        if target_pose is None:
            return False

        self.get_logger().info(
            f"IK target in '{self.planning_frame}': "
            f"position=({target_pose.pose.position.x:.4f}, "
            f"{target_pose.pose.position.y:.4f}, "
            f"{target_pose.pose.position.z:.4f}) m."
        )

        request = self._build_ik_request(
            target_pose,
            self._seed_positions(),
        )
        future = self.ik_client.call_async(request)
        if not self._wait_for_future(
            future,
            self.ik_timeout_sec + 3.0,
            "numerical IK",
        ):
            return False

        joint_goal = self._extract_joint_goal(future.result())
        if joint_goal is None:
            return False

        if self.print_joints:
            self.get_logger().info("Numerical IK solution:")
            for joint, value in zip(ARM_JOINTS, joint_goal):
                self.get_logger().info(
                    f"  {joint}: {math.degrees(value):.4f} deg "
                    f"({value:.6f} rad)"
                )

        if not self.execute_motion:
            self.get_logger().info(
                "execute:=false -> IK calculated but not executed."
            )
            return True

        self.get_logger().warn(
            "Executing without path planning, collision checking or "
            "singularity checking."
        )
        return self._execute_joint_goal(joint_goal)


def main():
    rclpy.init()
    node = ArmPoseNumericIK()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received. Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
