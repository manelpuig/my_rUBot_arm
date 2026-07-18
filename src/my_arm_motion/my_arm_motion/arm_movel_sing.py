#!/usr/bin/env python3
import math
import time
from threading import Thread

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

from my_arm_motion.arm_movel import ARM_JOINTS, ArmMoveL


class ArmMoveLSingularityChecked(ArmMoveL):
    """Straight Cartesian MoveL with singularity and joint-jump checks."""

    def __init__(self):
        super().__init__()

        self.declare_parameter("check_singularities", True)
        self.declare_parameter("jacobian_delta", 0.0001)
        self.declare_parameter("singularity_samples", 20)
        self.declare_parameter("min_singular_value", 0.01)
        self.declare_parameter("max_condition_number", 200.0)
        self.declare_parameter("max_joint_jump_deg", 45.0)

        self.check_singularities = bool(
            self.get_parameter("check_singularities").value
        )
        self.jacobian_delta = float(
            self.get_parameter("jacobian_delta").value
        )
        self.singularity_samples = int(
            self.get_parameter("singularity_samples").value
        )
        self.min_singular_value = float(
            self.get_parameter("min_singular_value").value
        )
        self.max_condition_number = float(
            self.get_parameter("max_condition_number").value
        )
        self.max_joint_jump_deg = float(
            self.get_parameter("max_joint_jump_deg").value
        )

        if self.jacobian_delta <= 0.0:
            raise ValueError("Parameter 'jacobian_delta' must be greater than zero.")
        if self.singularity_samples < 0:
            raise ValueError("Parameter 'singularity_samples' cannot be negative.")
        if self.min_singular_value < 0.0:
            raise ValueError("Parameter 'min_singular_value' cannot be negative.")
        if self.max_condition_number <= 1.0:
            raise ValueError(
                "Parameter 'max_condition_number' must be greater than one."
            )
        if self.max_joint_jump_deg <= 0.0:
            raise ValueError(
                "Parameter 'max_joint_jump_deg' must be greater than zero."
            )

        self.fk_client = self.create_client(
            GetPositionFK,
            "/compute_fk",
            callback_group=self.callback_group,
        )

    @staticmethod
    def _quaternion_to_array(quaternion):
        return np.array(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
            dtype=float,
        )

    @staticmethod
    def _quaternion_conjugate(quaternion):
        x, y, z, w = quaternion
        return np.array([-x, -y, -z, w], dtype=float)

    @staticmethod
    def _quaternion_multiply(left, right):
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return np.array(
            [
                lw * rx + lx * rw + ly * rz - lz * ry,
                lw * ry - lx * rz + ly * rw + lz * rx,
                lw * rz + lx * ry - ly * rx + lz * rw,
                lw * rw - lx * rx - ly * ry - lz * rz,
            ],
            dtype=float,
        )

    @classmethod
    def _rotation_vector_between(cls, initial, final):
        """Return the shortest rotation vector from initial to final quaternion."""
        initial = initial / np.linalg.norm(initial)
        final = final / np.linalg.norm(final)
        delta = cls._quaternion_multiply(
            final,
            cls._quaternion_conjugate(initial),
        )
        delta = delta / np.linalg.norm(delta)

        if delta[3] < 0.0:
            delta = -delta

        vector_norm = np.linalg.norm(delta[:3])
        if vector_norm < 1.0e-12:
            return np.zeros(3, dtype=float)

        angle = 2.0 * math.atan2(vector_norm, delta[3])
        return delta[:3] * (angle / vector_norm)

    def _build_fk_request(self, joint_positions):
        request = GetPositionFK.Request()
        request.header.frame_id = self.planning_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.fk_link_names = [self.ik_link]

        joint_state = JointState()
        joint_state.header = request.header
        joint_state.name = list(ARM_JOINTS)
        joint_state.position = [float(value) for value in joint_positions]
        request.robot_state.joint_state = joint_state

        return request

    def _compute_fk_pose(self, joint_positions):
        future = self.fk_client.call_async(
            self._build_fk_request(joint_positions)
        )
        if not self._wait_for_future(future, "FK request"):
            return None

        response = future.result()
        if response.error_code.val != response.error_code.SUCCESS:
            self.get_logger().error(
                f"FK failed with MoveIt error code: {response.error_code.val}"
            )
            return None
        if not response.pose_stamped:
            self.get_logger().error("FK response does not contain a tool pose.")
            return None

        return response.pose_stamped[0].pose

    def _compute_numerical_jacobian(self, joint_positions):
        base_pose = self._compute_fk_pose(joint_positions)
        if base_pose is None:
            return None

        base_position = np.array(
            [
                base_pose.position.x,
                base_pose.position.y,
                base_pose.position.z,
            ],
            dtype=float,
        )
        base_quaternion = self._quaternion_to_array(base_pose.orientation)

        jacobian = np.zeros((6, len(ARM_JOINTS)), dtype=float)

        for column in range(len(ARM_JOINTS)):
            perturbed = np.array(joint_positions, dtype=float)
            perturbed[column] += self.jacobian_delta

            perturbed_pose = self._compute_fk_pose(perturbed.tolist())
            if perturbed_pose is None:
                return None

            perturbed_position = np.array(
                [
                    perturbed_pose.position.x,
                    perturbed_pose.position.y,
                    perturbed_pose.position.z,
                ],
                dtype=float,
            )
            perturbed_quaternion = self._quaternion_to_array(
                perturbed_pose.orientation
            )

            jacobian[:3, column] = (
                perturbed_position - base_position
            ) / self.jacobian_delta
            jacobian[3:, column] = self._rotation_vector_between(
                base_quaternion,
                perturbed_quaternion,
            ) / self.jacobian_delta

        return jacobian

    def _trajectory_joint_positions(self, trajectory):
        name_to_index = {
            name: index for index, name in enumerate(trajectory.joint_names)
        }
        if not all(joint in name_to_index for joint in ARM_JOINTS):
            self.get_logger().error(
                "Cartesian trajectory does not contain all expected arm joints."
            )
            return None

        return [
            np.array(
                [point.positions[name_to_index[joint]] for joint in ARM_JOINTS],
                dtype=float,
            )
            for point in trajectory.points
        ]

    def _check_joint_jumps(self, positions):
        if len(positions) < 2:
            return True

        maximum_jump = 0.0
        maximum_segment = 0
        for index in range(len(positions) - 1):
            jump = float(
                np.max(np.abs(positions[index + 1] - positions[index]))
            )
            if jump > maximum_jump:
                maximum_jump = jump
                maximum_segment = index

        maximum_jump_deg = math.degrees(maximum_jump)
        self.get_logger().info(
            f"Maximum Cartesian joint jump: {maximum_jump_deg:.3f} deg "
            f"between points {maximum_segment} and {maximum_segment + 1}."
        )

        if maximum_jump_deg > self.max_joint_jump_deg:
            self.get_logger().error(
                f"Cartesian trajectory rejected: joint jump "
                f"{maximum_jump_deg:.3f} deg exceeds limit "
                f"{self.max_joint_jump_deg:.3f} deg."
            )
            return False

        return True

    def _sample_indices(self, number_of_points):
        if number_of_points <= 0:
            return []
        if self.singularity_samples == 0:
            return list(range(number_of_points))

        sample_count = min(self.singularity_samples, number_of_points)
        return sorted(
            set(
                np.linspace(
                    0,
                    number_of_points - 1,
                    sample_count,
                    dtype=int,
                ).tolist()
            )
        )

    def _check_singularity_metrics(self, trajectory):
        positions = self._trajectory_joint_positions(trajectory)
        if not positions:
            return False

        if not self._check_joint_jumps(positions):
            return False

        if not self.check_singularities:
            self.get_logger().warn("Jacobian singularity checking is disabled.")
            return True

        if not self.fk_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("The /compute_fk service is unavailable.")
            return False

        indices = self._sample_indices(len(positions))
        self.get_logger().info(
            f"Checking Jacobian singularity metrics at {len(indices)} "
            f"of {len(positions)} Cartesian trajectory points..."
        )

        worst_sigma_min = math.inf
        worst_condition = 0.0
        worst_index = 0

        for index in indices:
            jacobian = self._compute_numerical_jacobian(
                positions[index].tolist()
            )
            if jacobian is None:
                return False

            singular_values = np.linalg.svd(jacobian, compute_uv=False)
            sigma_min = float(np.min(singular_values))
            sigma_max = float(np.max(singular_values))
            condition_number = (
                math.inf if sigma_min < 1.0e-12 else sigma_max / sigma_min
            )

            if sigma_min < worst_sigma_min:
                worst_sigma_min = sigma_min
                worst_condition = condition_number
                worst_index = index

            if (
                sigma_min < self.min_singular_value
                or condition_number > self.max_condition_number
            ):
                self.get_logger().error(
                    f"Cartesian trajectory rejected near point {index}: "
                    f"sigma_min={sigma_min:.6f}, "
                    f"condition_number={condition_number:.3f}."
                )
                return False

        self.get_logger().info(
            f"Singularity check passed. Worst sampled point={worst_index}, "
            f"sigma_min={worst_sigma_min:.6f}, "
            f"condition_number={worst_condition:.3f}."
        )
        return True

    def run(self):
        time.sleep(self.startup_delay_sec)

        if not self._wait_for_joint_state():
            return False

        target_pose = self._build_target_pose()
        target_pose = self._transform_pose_to_planning_frame(target_pose)
        if target_pose is None:
            return False

        self.get_logger().info(
            f"MoveL+singularity target in '{self.planning_frame}': "
            f"position=({target_pose.pose.position.x:.4f}, "
            f"{target_pose.pose.position.y:.4f}, "
            f"{target_pose.pose.position.z:.4f}) m"
        )
        self.get_logger().info(
            f"Computing Cartesian path: max_step={self.max_step:.4f} m, "
            f"minimum_fraction={self.fraction_threshold:.2f}, "
            f"avoid_collisions={self.avoid_collisions}."
        )

        planning_future = self.moveit2.plan_async(
            pose=target_pose,
            target_link=self.ik_link,
            cartesian=True,
            max_step=self.max_step,
        )
        if planning_future is None:
            self.get_logger().error(
                "The /compute_cartesian_path service is unavailable."
            )
            return False
        if not self._wait_for_future(planning_future, "Cartesian planning"):
            return False

        response = planning_future.result()
        fraction = float(response.fraction)
        self.get_logger().info(
            f"Cartesian path completed fraction: {fraction:.3f} "
            f"({100.0 * fraction:.1f}%)."
        )

        trajectory = self.moveit2.get_trajectory(
            planning_future,
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

        if not self._check_singularity_metrics(trajectory):
            self.get_logger().error(
                "MoveL will not be executed because the Cartesian trajectory "
                "failed the singularity/jump checks."
            )
            return False

        if not self.execute_motion:
            self.get_logger().info(
                "execute:=false -> trajectory will not be executed."
            )
            return True

        self.get_logger().info(
            "Executing singularity-checked Cartesian trajectory..."
        )
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
                f"MoveL execution failed with MoveIt error code: "
                f"{error_code.val}"
            )
            return False

        self.get_logger().info(
            "Singularity-checked Cartesian trajectory execution succeeded."
        )
        return True


def main():
    rclpy.init()
    node = ArmMoveLSingularityChecked()

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
