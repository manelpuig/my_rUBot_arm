#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "trajectory_file",
            default_value="/tmp/my_arm_movej_trajectory.yaml",
        ),
        DeclareLaunchArgument("start_tolerance_deg", default_value="2.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]

    node = Node(
        package="my_arm_motion",
        executable="arm_execute_saved_exe",
        output="screen",
        parameters=[
            {
                "trajectory_file": LaunchConfiguration("trajectory_file"),
                "start_tolerance_deg": LaunchConfiguration(
                    "start_tolerance_deg"
                ),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(arguments + [node])
