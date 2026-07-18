#!/usr/bin/env python3
import copy
import math
from pathlib import Path
import time
from threading import Thread

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import MultiThreadedExecutor
import yaml

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from my_arm_motion.arm_movej import ARM_JOINTS, quat_from_rpy_zyx
from my_arm_motion.arm_movej_sing import ArmMoveJSingularityChecked


class ArmMotionSequence(ArmMoveJSingularityChecked):
    """Plan, check and optionally execute a YAML sequence of MoveJ and MoveL."""

    def __init__(self):
        super().__init__()

        self.declare_parameter("sequence_file", "ur5e_movej_movel.yaml")
        self.sequence_file = str(self.get_parameter("sequence_file").value)

    def _resolve_sequence_file(self):
        requested = Path(self.sequence_file).expanduser()
        candidates = []

        if requested.is_absolute():
            candidates.append(requested)
        else:
            candidates.append(Path.cwd() / requested)
            try:
                package_share = Path(
                    get_package_share_directory("my_arm_motion")
                )
                candidates.append(package_share / "config" / requested)
            except Exception as error:
                self.get_logger().warn(
                    f"Could not locate the my_arm_motion package share: {error}"
                )

        for candidate in candidates:
            if candidate.is_file():
                return candidate.resolve()

        searched = ", ".join(str(candidate) for candidate in candidates)
        self.get_logger().error(
            f"Sequence file '{self.sequence_file}' was not found. "
            f"Searched: {searched}"
        )
        return None

    def _load_sequence(self):
        sequence_path = self._resolve_sequence_file()
        if sequence_path is None:
            return None

        try:
            with sequence_path.open("r", encoding="utf-8") as stream:
                document = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError) as error:
            self.get_logger().error(
                f"Could not read YAML sequence '{sequence_path}': {error}"
            )
            return None

        if not isinstance(document, dict):
            self.get_logger().error("The YAML root must be a mapping.")
            return None

        defaults = document.get("defaults", {})
        motions = document.get("motions")

        if not isinstance(defaults, dict):
            self.get_logger().error("YAML field 'defaults' must be a mapping.")
            return None
        if not isinstance(motions, list) or not motions:
            self.get_logger().error(
                "YAML field 'motions' must be a non-empty list."
            )
            return None
        if not all(isinstance(motion, dict) for motion in motions):
            self.get_logger().error("Every YAML motion must be a mapping.")
            return None

        self.get_logger().info(
            f"Loaded sequence '{document.get('sequence_name', sequence_path.stem)}' "
            f"with {len(motions)} motions from '{sequence_path}'."
        )
        return defaults, motions

    @staticmethod
    def _three_floats(settings, field_name):
        value = settings.get(field_name)
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError(
                f"Field '{field_name}' must contain exactly three values."
            )
        return [float(item) for item in value]

    @staticmethod
    def _bounded_scale(settings, field_name, default_value):
        value = float(settings.get(field_name, default_value))
        if not 0.0 < value <= 1.0:
            raise ValueError(f"Field '{field_name}' must be in the range (0, 1].")
        return value

    def _build_pose_from_settings(self, settings):
        xyz_mm = self._three_floats(settings, "target_xyz_mm")
        rpy_deg = self._three_floats(settings, "target_rpy_deg")
        target_frame = str(settings.get("target_frame", self.target_frame))

        xyz_m = [value / 1000.0 for value in xyz_mm]
        rpy_rad = [math.radians(value) for value in rpy_deg]
        qx, qy, qz, qw = quat_from_rpy_zyx(*rpy_rad)

        pose = PoseStamped()
        pose.header.frame_id = target_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = xyz_m[0]
        pose.pose.position.y = xyz_m[1]
        pose.pose.position.z = xyz_m[2]
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return self._transform_pose_to_planning_frame(pose)

    @staticmethod
    def _positions_from_joint_state(joint_state):
        name_to_position = dict(zip(joint_state.name, joint_state.position))
        if not all(joint in name_to_position for joint in ARM_JOINTS):
            return None
        return [float(name_to_position[joint]) for joint in ARM_JOINTS]

    def _ik_seed_from_settings(self, settings, start_state):
        manual_seed = settings.get("ik_seed_joints_deg")
        if manual_seed is not None:
            if not isinstance(manual_seed, (list, tuple)) or len(manual_seed) != 6:
                raise ValueError(
                    "Field 'ik_seed_joints_deg' must contain exactly six values."
                )
            self.get_logger().info("Using YAML ik_seed_joints_deg as IK seed.")
            return [math.radians(float(value)) for value in manual_seed]

        positions = self._positions_from_joint_state(start_state)
        if positions is None:
            raise ValueError(
                "The start state does not contain all joints required for the IK seed."
            )
        self.get_logger().info("Using the previous/current state as IK seed.")
        return positions

    def _configure_motion(self, settings):
        velocity = self._bounded_scale(
            settings,
            "max_velocity",
            self.max_velocity,
        )
        acceleration = self._bounded_scale(
            settings,
            "max_acceleration",
            self.max_acceleration,
        )
        avoid_collisions = bool(
            settings.get("avoid_collisions", self.avoid_collisions)
        )

        self.moveit2.max_velocity = velocity
        self.moveit2.max_acceleration = acceleration
        self.avoid_collisions = avoid_collisions
        self.moveit2.cartesian_avoid_collisions = avoid_collisions

        return velocity, acceleration, avoid_collisions

    def _plan_movej(self, target_pose, settings, start_state):
        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("The /compute_ik service is unavailable.")
            return None

        try:
            seed_positions = self._ik_seed_from_settings(settings, start_state)
        except ValueError as error:
            self.get_logger().error(str(error))
            return None

        ik_request = self._build_ik_request(target_pose, seed_positions)
        ik_future = self.ik_client.call_async(ik_request)
        if not self._wait_for_future(ik_future, "IK request"):
            return None

        joint_goal = self._extract_joint_goal(ik_future.result())
        if joint_goal is None:
            return None

        if self.print_joints:
            self.get_logger().info("MoveJ joint goal:")
            for joint_name, value in zip(ARM_JOINTS, joint_goal):
                self.get_logger().info(
                    f"  {joint_name}: {math.degrees(value):.4f} deg "
                    f"({value:.4f} rad)"
                )

        self.get_logger().info(
            "Planning collision-aware MoveJ trajectory with OMPL..."
        )
        planning_future = self.moveit2.plan_async(
            joint_positions=joint_goal,
            joint_names=ARM_JOINTS,
            tolerance_joint_position=self.joint_tolerance,
            start_joint_state=start_state,
            cartesian=False,
        )
        if planning_future is None:
            self.get_logger().error("The MoveIt planning service is unavailable.")
            return None
        if not self._wait_for_future(planning_future, "MoveJ planning"):
            return None

        trajectory = self.moveit2.get_trajectory(
            planning_future,
            cartesian=False,
        )
        if trajectory is None or not trajectory.points:
            self.get_logger().error("MoveIt could not generate a MoveJ trajectory.")
            return None

        self.get_logger().info(
            f"MoveJ planning succeeded: {len(trajectory.points)} points."
        )
        return trajectory

    def _plan_movel(self, target_pose, settings, start_state):
        max_step = float(settings.get("max_step", 0.005))
        fraction_threshold = float(settings.get("fraction_threshold", 1.0))
        jump_threshold = float(settings.get("jump_threshold", 0.0))

        if max_step <= 0.0:
            self.get_logger().error("MoveL field 'max_step' must be positive.")
            return None
        if not 0.0 <= fraction_threshold <= 1.0:
            self.get_logger().error(
                "MoveL field 'fraction_threshold' must be in [0, 1]."
            )
            return None
        if jump_threshold < 0.0:
            self.get_logger().error(
                "MoveL field 'jump_threshold' cannot be negative."
            )
            return None

        self.moveit2.cartesian_jump_threshold = jump_threshold

        self.get_logger().info(
            f"Computing Cartesian MoveL: max_step={max_step:.4f} m, "
            f"minimum_fraction={fraction_threshold:.3f}, "
            f"avoid_collisions={self.avoid_collisions}."
        )
        planning_future = self.moveit2.plan_async(
            pose=target_pose,
            target_link=self.ik_link,
            start_joint_state=start_state,
            cartesian=True,
            max_step=max_step,
        )
        if planning_future is None:
            self.get_logger().error(
                "The /compute_cartesian_path service is unavailable."
            )
            return None
        if not self._wait_for_future(planning_future, "Cartesian planning"):
            return None

        response = planning_future.result()
        fraction = float(response.fraction)
        self.get_logger().info(
            f"Cartesian path completed fraction: {fraction:.3f} "
            f"({100.0 * fraction:.1f}%)."
        )

        trajectory = self.moveit2.get_trajectory(
            planning_future,
            cartesian=True,
            cartesian_fraction_threshold=fraction_threshold,
        )
        if trajectory is None or not trajectory.points:
            self.get_logger().error(
                "No acceptable complete Cartesian trajectory was generated."
            )
            return None

        self.get_logger().info(
            f"MoveL planning succeeded: {len(trajectory.points)} points."
        )
        return trajectory

    def _trajectory_end_state(self, trajectory):
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = list(trajectory.joint_names)
        state.position = [float(value) for value in trajectory.points[-1].positions]
        return state

    def _execute_trajectory(self, trajectory, motion_name):
        self.get_logger().info(f"Executing '{motion_name}'...")
        self.moveit2.execute(trajectory)

        if not self._wait_for_execution():
            return False

        error_code = self.moveit2.get_last_execution_error_code()
        if error_code is None:
            self.get_logger().error(
                "Execution finished without a MoveIt result code."
            )
            return False
        if error_code.val != error_code.SUCCESS:
            self.get_logger().error(
                f"Execution failed with MoveIt error code: {error_code.val}"
            )
            return False

        self.get_logger().info(f"Motion '{motion_name}' executed successfully.")
        return True

    def _live_state_or_fallback(self, fallback_state):
        live_state = self.moveit2.joint_state
        if live_state is None:
            return fallback_state
        if self._positions_from_joint_state(live_state) is None:
            return fallback_state
        return copy.deepcopy(live_state)

    def run(self):
        time.sleep(self.startup_delay_sec)

        if not self._wait_for_joint_state():
            return False

        loaded = self._load_sequence()
        if loaded is None:
            return False
        defaults, motions = loaded

        start_state = copy.deepcopy(self.moveit2.joint_state)
        execution_mode = "EXECUTE" if self.execute_motion else "PLAN ONLY"
        self.get_logger().info(f"Sequence mode: {execution_mode}.")

        for index, motion in enumerate(motions, start=1):
            settings = dict(defaults)
            settings.update(motion)

            motion_type = str(settings.get("motion_type", "")).strip().lower()
            motion_name = str(settings.get("name", f"motion_{index}"))
            if motion_type not in ("movej", "movel"):
                self.get_logger().error(
                    f"Motion {index} '{motion_name}' has invalid motion_type "
                    f"'{motion_type}'. Use 'movej' or 'movel'."
                )
                return False

            try:
                velocity, acceleration, avoid_collisions = (
                    self._configure_motion(settings)
                )
                target_pose = self._build_pose_from_settings(settings)
            except (TypeError, ValueError) as error:
                self.get_logger().error(
                    f"Invalid motion {index} '{motion_name}': {error}"
                )
                return False
            if target_pose is None:
                return False

            self.get_logger().info(
                f"--- Motion {index}/{len(motions)}: "
                f"{motion_name} [{motion_type.upper()}] ---"
            )
            self.get_logger().info(
                f"Target position=({target_pose.pose.position.x:.4f}, "
                f"{target_pose.pose.position.y:.4f}, "
                f"{target_pose.pose.position.z:.4f}) m, "
                f"velocity={velocity:.2f}, acceleration={acceleration:.2f}, "
                f"avoid_collisions={avoid_collisions}."
            )

            if motion_type == "movej":
                trajectory = self._plan_movej(
                    target_pose,
                    settings,
                    start_state,
                )
            else:
                trajectory = self._plan_movel(
                    target_pose,
                    settings,
                    start_state,
                )

            if trajectory is None:
                self.get_logger().error(
                    f"Sequence stopped: planning failed at motion {index} "
                    f"'{motion_name}'."
                )
                return False

            if not self._check_singularity_metrics(trajectory):
                self.get_logger().error(
                    f"Sequence stopped: motion {index} '{motion_name}' failed "
                    "the singularity/jump checks."
                )
                return False

            planned_end_state = self._trajectory_end_state(trajectory)

            if self.execute_motion:
                if not self._execute_trajectory(trajectory, motion_name):
                    self.get_logger().error(
                        f"Sequence stopped after execution failure at motion "
                        f"{index} '{motion_name}'."
                    )
                    return False

                wait_after_sec = float(settings.get("wait_after_sec", 0.0))
                if wait_after_sec < 0.0:
                    self.get_logger().error(
                        "Field 'wait_after_sec' cannot be negative."
                    )
                    return False
                if wait_after_sec > 0.0:
                    self.get_logger().info(
                        f"Waiting {wait_after_sec:.2f} s after '{motion_name}'."
                    )
                    time.sleep(wait_after_sec)

                start_state = self._live_state_or_fallback(planned_end_state)
            else:
                start_state = planned_end_state
                self.get_logger().info(
                    f"Motion '{motion_name}' validated without execution."
                )

        if self.execute_motion:
            self.get_logger().info(
                "Motion sequence completed and executed successfully."
            )
        else:
            self.get_logger().info(
                "Complete motion sequence planned and validated successfully."
            )
        return True


def main():
    rclpy.init()
    node = ArmMotionSequence()

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
