#!/usr/bin/env python3
from pathlib import Path

import yaml
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _duration_to_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


def _seconds_to_duration(seconds, duration):
    seconds = max(0.0, float(seconds))
    whole = int(seconds)
    duration.sec = whole
    duration.nanosec = int(round((seconds - whole) * 1.0e9))
    if duration.nanosec >= 1_000_000_000:
        duration.sec += 1
        duration.nanosec -= 1_000_000_000


def trajectory_to_dict(trajectory, metadata=None):
    return {
        "metadata": dict(metadata or {}),
        "trajectory": {
            "joint_names": list(trajectory.joint_names),
            "points": [
                {
                    "positions": [float(v) for v in point.positions],
                    "velocities": [float(v) for v in point.velocities],
                    "accelerations": [float(v) for v in point.accelerations],
                    "effort": [float(v) for v in point.effort],
                    "time_from_start": _duration_to_seconds(point.time_from_start),
                }
                for point in trajectory.points
            ],
        },
    }


def trajectory_from_dict(data):
    trajectory_data = data["trajectory"]
    trajectory = JointTrajectory()
    trajectory.joint_names = list(trajectory_data["joint_names"])

    for point_data in trajectory_data["points"]:
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in point_data.get("positions", [])]
        point.velocities = [float(v) for v in point_data.get("velocities", [])]
        point.accelerations = [
            float(v) for v in point_data.get("accelerations", [])
        ]
        point.effort = [float(v) for v in point_data.get("effort", [])]
        _seconds_to_duration(
            point_data.get("time_from_start", 0.0),
            point.time_from_start,
        )
        trajectory.points.append(point)

    return trajectory, dict(data.get("metadata", {}))


def save_trajectory_yaml(path, trajectory, metadata=None):
    output_path = Path(path).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(
            trajectory_to_dict(trajectory, metadata),
            stream,
            sort_keys=False,
            default_flow_style=False,
        )
    return output_path


def load_trajectory_yaml(path):
    input_path = Path(path).expanduser().resolve()
    with input_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    if not isinstance(data, dict) or "trajectory" not in data:
        raise ValueError("The YAML file does not contain a valid trajectory section.")

    trajectory, metadata = trajectory_from_dict(data)
    return input_path, trajectory, metadata
