#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("target_xyz", default_value="[0.40, 0.00, 0.50]"),
        DeclareLaunchArgument("target_rpy", default_value="[0.0, 3.14159, 0.0]"),
        DeclareLaunchArgument("max_step", default_value="0.005"),
        DeclareLaunchArgument("fraction_threshold", default_value="1.0"),
        DeclareLaunchArgument("candidate_attempts", default_value="3"),
        DeclareLaunchArgument(
            "max_step_scales",
            default_value="[1.0, 0.75, 0.5]",
        ),
        DeclareLaunchArgument("singularity_samples", default_value="20"),
        DeclareLaunchArgument("min_singular_value", default_value="0.01"),
        DeclareLaunchArgument("max_condition_number", default_value="200.0"),
        DeclareLaunchArgument("max_joint_jump_deg", default_value="45.0"),
        DeclareLaunchArgument(
            "trajectory_file",
            default_value="/tmp/my_arm_movel_trajectory.yaml",
        ),
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]

    node = Node(
        package="my_arm_motion",
        executable="arm_movel_candidates_exe",
        output="screen",
        parameters=[
            {
                "target_xyz": LaunchConfiguration("target_xyz"),
                "target_rpy": LaunchConfiguration("target_rpy"),
                "max_step": LaunchConfiguration("max_step"),
                "fraction_threshold": LaunchConfiguration(
                    "fraction_threshold"
                ),
                "candidate_attempts": LaunchConfiguration(
                    "candidate_attempts"
                ),
                "max_step_scales": LaunchConfiguration("max_step_scales"),
                "singularity_samples": LaunchConfiguration(
                    "singularity_samples"
                ),
                "min_singular_value": LaunchConfiguration(
                    "min_singular_value"
                ),
                "max_condition_number": LaunchConfiguration(
                    "max_condition_number"
                ),
                "max_joint_jump_deg": LaunchConfiguration(
                    "max_joint_jump_deg"
                ),
                "trajectory_file": LaunchConfiguration("trajectory_file"),
                "execute": LaunchConfiguration("execute"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(arguments + [node])
