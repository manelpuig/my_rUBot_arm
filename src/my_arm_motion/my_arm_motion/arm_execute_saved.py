#!/usr/bin/env python3
import math
import time
from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor

from my_arm_motion.arm_movej import ARM_JOINTS, ArmMoveJ
from my_arm_motion.trajectory_storage import load_trajectory_yaml


class ArmExecuteSaved(ArmMoveJ):
    """Load and execute a previously saved JointTrajectory without replanning."""

    def __init__(self):
        super().__init__()
        self._name = "arm_execute_saved"

        self.declare_parameter(
            "trajectory_file",
            "/tmp/my_arm_movej_trajectory.yaml",
        )
        self.declare_parameter("start_tolerance_deg", 2.0)
        self.declare_parameter("require_same_joint_order", True)

        self.trajectory_file = str(self.get_parameter("trajectory_file").value)
        self.start_tolerance_deg = float(
            self.get_parameter("start_tolerance_deg").value
        )
        self.require_same_joint_order = bool(
            self.get_parameter("require_same_joint_order").value
        )

        if self.start_tolerance_deg <= 0.0:
            raise ValueError("'start_tolerance_deg' must be greater than zero.")

    def _current_positions(self):
        state = self.moveit2.joint_state
        mapping = dict(zip(state.name, state.position))
        if not all(joint in mapping for joint in ARM_JOINTS):
            return None
        return [float(mapping[joint]) for joint in ARM_JOINTS]

    def _trajectory_start_for_arm(self, trajectory):
        if not trajectory.points:
            return None

        if self.require_same_joint_order and list(trajectory.joint_names) != list(ARM_JOINTS):
            self.get_logger().error(
                "Saved trajectory joint order does not match ARM_JOINTS."
            )
            return None

        index = {name: i for i, name in enumerate(trajectory.joint_names)}
        if not all(joint in index for joint in ARM_JOINTS):
            self.get_logger().error(
                "Saved trajectory does not contain all expected arm joints."
            )
            return None

        point = trajectory.points[0]
        return [float(point.positions[index[joint]]) for joint in ARM_JOINTS]

    def run(self):
        time.sleep(self.startup_delay_sec)

        if not self._wait_for_joint_state():
            return False

        try:
            path, trajectory, metadata = load_trajectory_yaml(
                self.trajectory_file
            )
        except Exception as error:
            self.get_logger().error(
                f"Could not load saved trajectory: {error}"
            )
            return False

        if not trajectory.points:
            self.get_logger().error("Saved trajectory contains no points.")
            return False

        current = self._current_positions()
        planned_start = self._trajectory_start_for_arm(trajectory)
        if current is None or planned_start is None:
            return False

        errors_deg = [
            abs(math.degrees(a - b))
            for a, b in zip(current, planned_start)
        ]
        maximum_error = max(errors_deg)

        self.get_logger().info(
            f"Loaded {len(trajectory.points)} points from {path}."
        )
        if metadata:
            self.get_logger().info(
                f"Saved motion type: {metadata.get('motion_type', 'unknown')}."
            )
        self.get_logger().info(
            f"Maximum start-state error: {maximum_error:.3f} deg."
        )

        if maximum_error > self.start_tolerance_deg:
            self.get_logger().error(
                f"Execution rejected: start-state error {maximum_error:.3f} deg "
                f"exceeds {self.start_tolerance_deg:.3f} deg. "
                "Move the robot to the original start state or replan."
            )
            return False

        self.get_logger().warn(
            "Executing without replanning. The saved path is not automatically "
            "updated for changes in the Planning Scene."
        )
        self.moveit2.execute(trajectory)

        if not self._wait_for_execution():
            return False

        error_code = self.moveit2.get_last_execution_error_code()
        if error_code is None or error_code.val != error_code.SUCCESS:
            self.get_logger().error(
                "Saved trajectory execution failed."
            )
            return False

        self.get_logger().info(
            "Saved trajectory execution succeeded without replanning."
        )
        return True


def main():
    rclpy.init()
    node = ArmExecuteSaved()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received. Shutting down...")
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
