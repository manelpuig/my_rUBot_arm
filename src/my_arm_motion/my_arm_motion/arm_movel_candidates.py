#!/usr/bin/env python3
import math
import time
from threading import Thread

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from my_arm_motion.arm_movel_sing import ArmMoveLSingularityChecked
from my_arm_motion.trajectory_storage import save_trajectory_yaml


class ArmMoveLCandidates(ArmMoveLSingularityChecked):
    """Generate Cartesian MoveL candidates, select the best and save it."""

    def __init__(self):
        super().__init__()
        self._name = "arm_movel_candidates"

        self.declare_parameter("candidate_attempts", 3)
        self.declare_parameter(
            "max_step_scales",
            [1.0, 0.75, 0.5],
        )
        self.declare_parameter(
            "trajectory_file",
            "/tmp/my_arm_movel_trajectory.yaml",
        )
        self.declare_parameter("save_trajectory", True)

        self.candidate_attempts = int(
            self.get_parameter("candidate_attempts").value
        )
        self.max_step_scales = [
            float(v) for v in self.get_parameter("max_step_scales").value
        ]
        self.trajectory_file = str(self.get_parameter("trajectory_file").value)
        self.save_trajectory = bool(
            self.get_parameter("save_trajectory").value
        )

        if self.candidate_attempts < 1:
            raise ValueError("'candidate_attempts' must be at least one.")
        if not self.max_step_scales or any(v <= 0.0 for v in self.max_step_scales):
            raise ValueError("'max_step_scales' must contain positive values.")

    @staticmethod
    def _joint_path_length(positions):
        if len(positions) < 2:
            return 0.0
        return float(
            sum(
                np.linalg.norm(positions[i + 1] - positions[i])
                for i in range(len(positions) - 1)
            )
        )

    def _evaluate_trajectory(self, trajectory, fraction):
        positions = self._trajectory_joint_positions(trajectory)
        if not positions:
            return None

        max_jump = 0.0
        for index in range(len(positions) - 1):
            max_jump = max(
                max_jump,
                float(np.max(np.abs(positions[index + 1] - positions[index]))),
            )
        max_jump_deg = math.degrees(max_jump)
        if max_jump_deg > self.max_joint_jump_deg:
            return None

        metrics = {
            "fraction": float(fraction),
            "min_sigma": math.inf,
            "max_condition": 0.0,
            "max_joint_jump_deg": max_jump_deg,
            "joint_path_length": self._joint_path_length(positions),
            "worst_index": 0,
        }

        if self.check_singularities:
            if not self.fk_client.wait_for_service(timeout_sec=3.0):
                return None

            for index in self._sample_indices(len(positions)):
                jacobian = self._compute_numerical_jacobian(
                    positions[index].tolist()
                )
                if jacobian is None:
                    return None

                singular_values = np.linalg.svd(jacobian, compute_uv=False)
                sigma_min = float(np.min(singular_values))
                sigma_max = float(np.max(singular_values))
                condition = (
                    math.inf
                    if sigma_min < 1.0e-12
                    else sigma_max / sigma_min
                )

                if sigma_min < metrics["min_sigma"]:
                    metrics["min_sigma"] = sigma_min
                    metrics["worst_index"] = index
                metrics["max_condition"] = max(
                    metrics["max_condition"],
                    condition,
                )

                if (
                    sigma_min < self.min_singular_value
                    or condition > self.max_condition_number
                ):
                    return None
        else:
            metrics["min_sigma"] = 1.0
            metrics["max_condition"] = 1.0

        metrics["score"] = (
            1000.0 * metrics["fraction"]
            + 100.0 * metrics["min_sigma"]
            - 0.01 * metrics["max_condition"]
            - 0.10 * metrics["joint_path_length"]
            - 0.01 * metrics["max_joint_jump_deg"]
        )
        return metrics

    def _plan_candidate(self, target_pose, max_step):
        original_max_step = self.max_step
        self.max_step = max_step
        try:
            future = self.moveit2.plan_async(
                pose=target_pose,
                target_link=self.ik_link,
                cartesian=True,
                max_step=max_step,
            )
            if future is None:
                return None, 0.0
            if not self._wait_for_future(future, "Cartesian candidate planning"):
                return None, 0.0

            response = future.result()
            fraction = float(response.fraction)
            trajectory = self.moveit2.get_trajectory(
                future,
                cartesian=True,
                cartesian_fraction_threshold=self.fraction_threshold,
            )
            return trajectory, fraction
        finally:
            self.max_step = original_max_step

    def run(self):
        time.sleep(self.startup_delay_sec)

        if not self._wait_for_joint_state():
            return False

        target_pose = self._transform_pose_to_planning_frame(
            self._build_target_pose()
        )
        if target_pose is None:
            return False

        valid_candidates = []
        candidate_number = 0

        for scale in self.max_step_scales:
            candidate_step = self.max_step * scale
            for attempt in range(self.candidate_attempts):
                candidate_number += 1
                trajectory, fraction = self._plan_candidate(
                    target_pose,
                    candidate_step,
                )
                if trajectory is None or not trajectory.points:
                    self.get_logger().warn(
                        f"Candidate {candidate_number}: no acceptable Cartesian path "
                        f"(fraction={fraction:.3f}, max_step={candidate_step:.5f})."
                    )
                    continue

                metrics = self._evaluate_trajectory(trajectory, fraction)
                if metrics is None:
                    self.get_logger().warn(
                        f"Candidate {candidate_number}: rejected by "
                        "singularity/jump checks."
                    )
                    continue

                valid_candidates.append(
                    {
                        "trajectory": trajectory,
                        "metrics": metrics,
                        "max_step": candidate_step,
                        "attempt": attempt + 1,
                    }
                )
                self.get_logger().info(
                    f"Candidate {candidate_number}: valid, "
                    f"fraction={fraction:.3f}, max_step={candidate_step:.5f}, "
                    f"sigma_min={metrics['min_sigma']:.6f}, "
                    f"condition={metrics['max_condition']:.2f}, "
                    f"score={metrics['score']:.3f}."
                )

        if not valid_candidates:
            self.get_logger().error(
                "No singularity-safe MoveL candidate found. "
                "A strict straight line may have no valid alternative."
            )
            return False

        best = max(valid_candidates, key=lambda candidate: candidate["metrics"]["score"])
        trajectory = best["trajectory"]
        metrics = best["metrics"]

        self.get_logger().info(
            f"Selected MoveL candidate: max_step={best['max_step']:.5f}, "
            f"attempt={best['attempt']}, fraction={metrics['fraction']:.3f}, "
            f"score={metrics['score']:.3f}."
        )

        if self.save_trajectory:
            output = save_trajectory_yaml(
                self.trajectory_file,
                trajectory,
                {
                    "motion_type": "MoveL",
                    "planning_frame": self.planning_frame,
                    "group_name": self.group_name,
                    "ik_link": self.ik_link,
                    "target_xyz": self.target_xyz,
                    "target_rpy": self.target_rpy,
                    "max_step": best["max_step"],
                    "attempt": best["attempt"],
                    **metrics,
                },
            )
            self.get_logger().info(f"Saved selected trajectory to: {output}")

        if not self.execute_motion:
            self.get_logger().info(
                "execute:=false -> selected trajectory was saved but not executed."
            )
            return True

        self.get_logger().info("Executing selected MoveL trajectory...")
        self.moveit2.execute(trajectory)
        if not self._wait_for_execution():
            return False

        error_code = self.moveit2.get_last_execution_error_code()
        if error_code is None or error_code.val != error_code.SUCCESS:
            self.get_logger().error("Selected MoveL trajectory execution failed.")
            return False

        self.get_logger().info("Selected MoveL trajectory execution succeeded.")
        return True


def main():
    rclpy.init()
    node = ArmMoveLCandidates()
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
