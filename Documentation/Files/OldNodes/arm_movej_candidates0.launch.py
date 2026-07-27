#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    target_xyz = DeclareLaunchArgument(
        "target_xyz",
        default_value="[400.0,0.0,500.0]",
        description="MoveJ target position [mm] as [x,y,z]",
    )
    target_xyz_m = PythonExpression([
        "[x/1000.0 for x in ",
        LaunchConfiguration("target_xyz"),
        "]",
    ])

    target_rpy = DeclareLaunchArgument(
        "target_rpy",
        default_value="[0.0,180.0,0.0]",
        description="MoveJ target orientation [deg] as [roll,pitch,yaw]",
    )
    target_rpy_rad = PythonExpression([
        "[x*3.141592653589793/180.0 for x in ",
        LaunchConfiguration("target_rpy"),
        "]",
    ])

    arguments = [
        DeclareLaunchArgument("ik_candidates", default_value="4"),
        DeclareLaunchArgument("plans_per_ik", default_value="3"),
        DeclareLaunchArgument("seed_perturbation_deg", default_value="90.0"),
        DeclareLaunchArgument("check_singularities", default_value="true"),
        DeclareLaunchArgument("avoid_collisions", default_value="true"),
        DeclareLaunchArgument("singularity_samples", default_value="20"),
        DeclareLaunchArgument("min_singular_value", default_value="0.01"),
        DeclareLaunchArgument("max_condition_number", default_value="200.0"),
        DeclareLaunchArgument("max_joint_jump_deg", default_value="45.0"),
        DeclareLaunchArgument(
            "trajectory_file",
            default_value="/tmp/my_arm_movej_trajectory.yaml",
        ),
        DeclareLaunchArgument("save_trajectory", default_value="true"),
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]

    node = Node(
        package="my_arm_motion",
        executable="arm_movej_candidates_exe",
        name="arm_movej_candidates",
        output="screen",
        parameters=[{
            "target_xyz": target_xyz_m,
            "target_rpy": target_rpy_rad,
            "ik_candidates": LaunchConfiguration("ik_candidates"),
            "plans_per_ik": LaunchConfiguration("plans_per_ik"),
            "seed_perturbation_deg": LaunchConfiguration(
                "seed_perturbation_deg"
            ),
            "check_singularities": LaunchConfiguration(
                "check_singularities"
            ),
            "avoid_collisions": LaunchConfiguration("avoid_collisions"),
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
            "save_trajectory": LaunchConfiguration("save_trajectory"),
            "execute": LaunchConfiguration("execute"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription([
        target_xyz,
        target_rpy,
        *arguments,
        node,
    ])
