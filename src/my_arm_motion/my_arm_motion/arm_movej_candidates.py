#!/usr/bin/env python3
import math
import random
import time
from threading import Thread

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from my_arm_motion.arm_movej import ARM_JOINTS
from my_arm_motion.arm_movej_sing import ArmMoveJSingularityChecked
from my_arm_motion.trajectory_storage import save_trajectory_yaml


class ArmMoveJCandidates(ArmMoveJSingularityChecked):
    """Generate IK/OMPL MoveJ candidates, select the best and save it."""

    def __init__(self):
        super().__init__()
        self._name = "arm_movej_candidates"

        self.declare_parameter("ik_candidates", 4)
        self.declare_parameter("plans_per_ik", 3)
        self.declare_parameter("seed_perturbation_deg", 90.0)
        self.declare_parameter("random_seed", 7)
        self.declare_parameter(
            "trajectory_file",
            "/tmp/my_arm_movej_trajectory.yaml",
        )
        self.declare_parameter("save_trajectory", True)

        self.ik_candidates = int(self.get_parameter("ik_candidates").value)
        self.plans_per_ik = int(self.get_parameter("plans_per_ik").value)
        self.seed_perturbation_deg = float(
            self.get_parameter("seed_perturbation_deg").value
        )
        self.random_seed = int(self.get_parameter("random_seed").value)
        self.trajectory_file = str(self.get_parameter("trajectory_file").value)
        self.save_trajectory = bool(
            self.get_parameter("save_trajectory").value
        )

        if self.ik_candidates < 1:
            raise ValueError("'ik_candidates' must be at least one.")
        if self.plans_per_ik < 1:
            raise ValueError("'plans_per_ik' must be at least one.")
        if self.seed_perturbation_deg < 0.0:
            raise ValueError("'seed_perturbation_deg' cannot be negative.")

        self._rng = random.Random(self.random_seed)

    def _candidate_seed(self, base_seed, index):
        if index == 0:
            return list(base_seed)

        amplitude = math.radians(self.seed_perturbation_deg)
        seed = []
        for value in base_seed:
            perturbed = float(value) + self._rng.uniform(-amplitude, amplitude)
            seed.append(math.atan2(math.sin(perturbed), math.cos(perturbed)))
        return seed

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

    def _evaluate_trajectory(self, trajectory):
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
            "min_sigma": math.inf,
            "max_condition": 0.0,
            "max_joint_jump_deg": max_jump_deg,
            "joint_path_length": self._joint_path_length(positions),
            "worst_index": 0,
        }

        if self.check_singularities:
            if not self.fk_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error("The /compute_fk service is unavailable.")
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
            100.0 * metrics["min_sigma"]
            - 0.01 * metrics["max_condition"]
            - 0.10 * metrics["joint_path_length"]
            - 0.01 * metrics["max_joint_jump_deg"]
        )
        return metrics

    @staticmethod
    def _same_goal(first, second, tolerance=1.0e-3):
        return max(abs(a - b) for a, b in zip(first, second)) <= tolerance

    def _compute_ik_candidate(self, target_pose, seed):
        request = self._build_ik_request(target_pose, seed)
        future = self.ik_client.call_async(request)
        if not self._wait_for_future(future, "IK candidate request"):
            return None
        return self._extract_joint_goal(future.result())

    def _plan_candidate(self, current_state, joint_goal):
        future = self.moveit2.plan_async(
            joint_positions=joint_goal,
            joint_names=ARM_JOINTS,
            tolerance_joint_position=self.joint_tolerance,
            start_joint_state=current_state,
            cartesian=False,
        )
        if future is None:
            return None
        if not self._wait_for_future(future, "OMPL candidate planning"):
            return None
        return self.moveit2.get_trajectory(future, cartesian=False)

    def run(self):
        time.sleep(self.startup_delay_sec)

        if not self._wait_for_joint_state():
            return False
        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("The /compute_ik service is unavailable.")
            return False

        current_state = self.moveit2.joint_state
        target_pose = self._transform_pose_to_planning_frame(
            self._build_target_pose()
        )
        if target_pose is None:
            return False

        base_seed = self._get_seed_positions(current_state)
        unique_goals = []
        valid_candidates = []
        candidate_number = 0

        self.get_logger().info(
            f"Searching up to {self.ik_candidates} IK candidates and "
            f"{self.plans_per_ik} OMPL plans per IK."
        )

        for ik_index in range(self.ik_candidates):
            seed = self._candidate_seed(base_seed, ik_index)
            joint_goal = self._compute_ik_candidate(target_pose, seed)
            if joint_goal is None:
                self.get_logger().warn(
                    f"IK candidate {ik_index + 1} failed."
                )
                continue

            if any(self._same_goal(joint_goal, old) for old in unique_goals):
                self.get_logger().info(
                    f"IK candidate {ik_index + 1} duplicates a previous solution."
                )
                continue
            unique_goals.append(joint_goal)

            for plan_index in range(self.plans_per_ik):
                candidate_number += 1
                trajectory = self._plan_candidate(current_state, joint_goal)
                if trajectory is None or not trajectory.points:
                    self.get_logger().warn(
                        f"Candidate {candidate_number}: planning failed."
                    )
                    continue

                metrics = self._evaluate_trajectory(trajectory)
                if metrics is None:
                    self.get_logger().warn(
                        f"Candidate {candidate_number}: rejected."
                    )
                    continue

                valid_candidates.append(
                    {
                        "trajectory": trajectory,
                        "metrics": metrics,
                        "ik_index": ik_index + 1,
                        "plan_index": plan_index + 1,
                    }
                )
                self.get_logger().info(
                    f"Candidate {candidate_number}: valid, "
                    f"sigma_min={metrics['min_sigma']:.6f}, "
                    f"condition={metrics['max_condition']:.2f}, "
                    f"path={metrics['joint_path_length']:.3f}, "
                    f"score={metrics['score']:.3f}."
                )

        if not valid_candidates:
            self.get_logger().error(
                "No collision-free and singularity-safe MoveJ candidate found."
            )
            return False

        best = max(valid_candidates, key=lambda candidate: candidate["metrics"]["score"])
        trajectory = best["trajectory"]
        metrics = best["metrics"]

        self.get_logger().info(
            f"Selected IK {best['ik_index']}, plan {best['plan_index']}: "
            f"score={metrics['score']:.3f}, "
            f"sigma_min={metrics['min_sigma']:.6f}, "
            f"condition={metrics['max_condition']:.2f}."
        )

        if self.save_trajectory:
            output = save_trajectory_yaml(
                self.trajectory_file,
                trajectory,
                {
                    "motion_type": "MoveJ",
                    "planning_frame": self.planning_frame,
                    "group_name": self.group_name,
                    "ik_link": self.ik_link,
                    "target_xyz": self.target_xyz,
                    "target_rpy": self.target_rpy,
                    "ik_candidate": best["ik_index"],
                    "ompl_plan": best["plan_index"],
                    **metrics,
                },
            )
            self.get_logger().info(f"Saved selected trajectory to: {output}")

        if not self.execute_motion:
            self.get_logger().info(
                "execute:=false -> selected trajectory was saved but not executed."
            )
            return True

        self.get_logger().info("Executing selected MoveJ trajectory...")
        self.moveit2.execute(trajectory)
        if not self._wait_for_execution():
            return False

        error_code = self.moveit2.get_last_execution_error_code()
        if error_code is None or error_code.val != error_code.SUCCESS:
            self.get_logger().error("Selected MoveJ trajectory execution failed.")
            return False

        self.get_logger().info("Selected MoveJ trajectory execution succeeded.")
        return True


def main():
    rclpy.init()
    node = ArmMoveJCandidates()
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
