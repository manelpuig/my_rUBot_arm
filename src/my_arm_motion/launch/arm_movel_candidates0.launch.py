#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    target_xyz = DeclareLaunchArgument(
        "target_xyz",
        default_value="[400.0, 0.0, 500.0]",
        description="MoveL target position [mm] as [x,y,z]",
    )

    target_xyz_m = PythonExpression([
        "[x / 1000.0 for x in ",
        LaunchConfiguration("target_xyz"),
        "]",
    ])

    target_rpy = DeclareLaunchArgument(
        "target_rpy",
        default_value="[0.0, 180.0, 0.0]",
        description="MoveL target orientation [deg] as [roll,pitch,yaw]",
    )

    target_rpy_rad = PythonExpression([
        "[x * 3.141592653589793 / 180.0 for x in ",
        LaunchConfiguration("target_rpy"),
        "]",
    ])

    arguments = [
        DeclareLaunchArgument(
            "max_step",
            default_value="0.005",
            description="Maximum Cartesian interpolation step [m]",
        ),
        DeclareLaunchArgument(
            "fraction_threshold",
            default_value="1.0",
            description="Minimum accepted Cartesian path fraction",
        ),
        DeclareLaunchArgument(
            "candidate_attempts",
            default_value="3",
            description="Number of attempts for each max_step value",
        ),
        DeclareLaunchArgument(
            "max_step_scales",
            default_value="[1.0, 0.75, 0.5]",
            description="Scale factors applied to max_step",
        ),
        DeclareLaunchArgument(
            "check_singularities",
            default_value="true",
            description="Enable Jacobian singularity analysis",
        ),
        DeclareLaunchArgument(
            "avoid_collisions",
            default_value="true",
            description="Enable collision checking during Cartesian planning",
        ),
        DeclareLaunchArgument(
            "singularity_samples",
            default_value="20",
            description="Number of trajectory samples used for singularity checking",
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
            "trajectory_file",
            default_value="/tmp/my_arm_movel_trajectory.yaml",
            description="Output YAML trajectory file",
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
        ),
    ]

    node = Node(
        package="my_arm_motion",
        executable="arm_movel_candidates_exe",
        name="arm_movel_candidates",
        output="screen",
        parameters=[
            {
                "target_xyz": target_xyz_m,
                "target_rpy": target_rpy_rad,
                "max_step": LaunchConfiguration("max_step"),
                "fraction_threshold": LaunchConfiguration(
                    "fraction_threshold"
                ),
                "candidate_attempts": LaunchConfiguration(
                    "candidate_attempts"
                ),
                "max_step_scales": LaunchConfiguration(
                    "max_step_scales"
                ),
                "check_singularities": LaunchConfiguration(
                    "check_singularities"
                ),
                "avoid_collisions": LaunchConfiguration(
                    "avoid_collisions"
                ),
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
                "trajectory_file": LaunchConfiguration(
                    "trajectory_file"
                ),
                "save_trajectory": LaunchConfiguration(
                    "save_trajectory"
                ),
                "execute": LaunchConfiguration("execute"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription([
        target_xyz,
        target_rpy,
        *arguments,
        node,
    ])