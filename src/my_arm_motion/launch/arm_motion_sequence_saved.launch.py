#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    trajectory_path = PathJoinSubstitution([
        FindPackageShare("my_arm_motion"),
        "trajectories",
        LaunchConfiguration("trajectory_filename"),
    ])

    arguments = [
        DeclareLaunchArgument(
            "sequence_file",
            default_value="ur5e_handshake_save.yaml",
            description=(
                "YAML sequence basename from config/ or an absolute path"
            ),
        ),
        DeclareLaunchArgument(
            "trajectory_filename",
            default_value="ur5e_handshake_planned.yaml",
            description=(
                "Name of the concatenated YAML trajectory saved inside "
                "the package trajectories directory"
            ),
        ),
        DeclareLaunchArgument(
            "save_trajectory",
            default_value="true",
            description="Save the concatenated trajectory to YAML",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "execute",
            default_value="false",
            description=(
                "Execute the concatenated trajectory after planning, "
                "validation and optional saving"
            ),
        ),
        DeclareLaunchArgument(
            "avoid_collisions",
            default_value="true",
            description="Enable collision checking during planning",
        ),
        DeclareLaunchArgument(
            "max_velocity",
            default_value="0.1",
            description="Default maximum velocity scaling factor",
        ),
        DeclareLaunchArgument(
            "max_acceleration",
            default_value="0.1",
            description="Default maximum acceleration scaling factor",
        ),
        DeclareLaunchArgument(
            "ik_timeout_sec",
            default_value="0.5",
            description="Maximum time allowed for each IK request [s]",
        ),
        DeclareLaunchArgument(
            "joint_tolerance",
            default_value="0.001",
            description="MoveJ joint-goal planning tolerance [rad]",
        ),
        DeclareLaunchArgument(
            "motion_timeout_sec",
            default_value="180.0",
            description="Maximum planning or execution wait time [s]",
        ),
        DeclareLaunchArgument(
            "check_singularities",
            default_value="true",
            description="Enable numerical Jacobian singularity analysis",
        ),
        DeclareLaunchArgument(
            "jacobian_delta",
            default_value="0.0001",
            description="Joint perturbation used for numerical Jacobian [rad]",
        ),
        DeclareLaunchArgument(
            "singularity_samples",
            default_value="20",
            description=(
                "Maximum sampled points per motion; 0 checks every point"
            ),
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
    ]

    node = Node(
        package="my_arm_motion",
        executable="arm_motion_sequence_saved_exe",
        name="arm_motion_sequence_saved",
        output="screen",
        parameters=[{
            "group_name": "arm",
            "ik_link": "tool",
            "target_frame": "base_link",
            "planning_frame": "base_link",

            "sequence_file": LaunchConfiguration("sequence_file"),
            "trajectory_file": trajectory_path,
            "save_trajectory": LaunchConfiguration("save_trajectory"),

            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "execute": LaunchConfiguration("execute"),
            "avoid_collisions": LaunchConfiguration("avoid_collisions"),
            "max_velocity": LaunchConfiguration("max_velocity"),
            "max_acceleration": LaunchConfiguration("max_acceleration"),
            "ik_timeout_sec": LaunchConfiguration("ik_timeout_sec"),
            "joint_tolerance": LaunchConfiguration("joint_tolerance"),
            "motion_timeout_sec": LaunchConfiguration(
                "motion_timeout_sec"
            ),

            "check_singularities": LaunchConfiguration(
                "check_singularities"
            ),
            "jacobian_delta": LaunchConfiguration("jacobian_delta"),
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
        }],
    )

    return LaunchDescription(arguments + [node])
