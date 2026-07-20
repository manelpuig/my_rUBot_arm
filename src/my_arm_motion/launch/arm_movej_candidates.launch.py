#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    target_xyz = DeclareLaunchArgument(
        "target_xyz",
        default_value="[400.0, 0.0, 500.0]",
        description="MoveJ target position [mm] as [x,y,z]",
    )

    target_rpy = DeclareLaunchArgument(
        "target_rpy",
        default_value="[0.0, 180.0, 0.0]",
        description="MoveJ target orientation [deg] as [roll,pitch,yaw]",
    )

    target_xyz_m = PythonExpression([
        "[x / 1000.0 for x in ",
        LaunchConfiguration("target_xyz"),
        "]",
    ])

    target_rpy_rad = PythonExpression([
        "[x * 3.141592653589793 / 180.0 for x in ",
        LaunchConfiguration("target_rpy"),
        "]",
    ])

    trajectory_filename = DeclareLaunchArgument(
        "trajectory_filename",
        default_value="movej_without_obstacle.yaml",
        description="Name of the output YAML trajectory file",
    )

    trajectory_path = PathJoinSubstitution([
        FindPackageShare("my_arm_motion"),
        "trajectories",
        LaunchConfiguration("trajectory_filename"),
    ])

    arguments = [
        target_xyz,
        target_rpy,
        trajectory_filename,
        DeclareLaunchArgument(
            "ik_candidates",
            default_value="4",
            description="Maximum number of IK candidates",
        ),
        DeclareLaunchArgument(
            "plans_per_ik",
            default_value="3",
            description="Number of OMPL planning attempts per IK candidate",
        ),
        DeclareLaunchArgument(
            "seed_perturbation_deg",
            default_value="90.0",
            description="Maximum random IK seed perturbation [deg]",
        ),
        DeclareLaunchArgument(
            "check_singularities",
            default_value="true",
            description="Enable Jacobian singularity analysis",
        ),
        DeclareLaunchArgument(
            "avoid_collisions",
            default_value="true",
            description="Enable collision checking during planning",
        ),
        DeclareLaunchArgument(
            "singularity_samples",
            default_value="20",
            description="Number of trajectory samples used for singularity analysis",
        ),
        DeclareLaunchArgument(
            "min_singular_value",
            default_value="0.01",
            description="Minimum accepted Jacobian singular value",
        ),
        DeclareLaunchArgument(
            "max_condition_number",
            default_value="200.0",
            description="Maximum accepted Jacobian condition number",
        ),
        DeclareLaunchArgument(
            "max_joint_jump_deg",
            default_value="45.0",
            description="Maximum accepted joint jump [deg]",
        ),
        DeclareLaunchArgument(
            "save_trajectory",
            default_value="true",
            description="Save the selected trajectory to YAML",
        ),
        DeclareLaunchArgument(
            "execute",
            default_value="false",
            description="Execute the selected trajectory",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
    ]

    node = Node(
        package="my_arm_motion",
        executable="arm_movej_candidates_exe",
        name="arm_movej_candidates",
        output="screen",
        parameters=[{
            "target_xyz": target_xyz_m,
            "target_rpy": target_rpy_rad,
            "trajectory_file": trajectory_path,
            "ik_candidates": LaunchConfiguration("ik_candidates"),
            "plans_per_ik": LaunchConfiguration("plans_per_ik"),
            "seed_perturbation_deg": LaunchConfiguration("seed_perturbation_deg"),
            "check_singularities": LaunchConfiguration("check_singularities"),
            "avoid_collisions": LaunchConfiguration("avoid_collisions"),
            "singularity_samples": LaunchConfiguration("singularity_samples"),
            "min_singular_value": LaunchConfiguration("min_singular_value"),
            "max_condition_number": LaunchConfiguration("max_condition_number"),
            "max_joint_jump_deg": LaunchConfiguration("max_joint_jump_deg"),
            "save_trajectory": LaunchConfiguration("save_trajectory"),
            "execute": LaunchConfiguration("execute"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription(arguments + [node])
