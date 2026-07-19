#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_trajectory_file = PathJoinSubstitution([
        FindPackageShare("my_arm_motion"),
        "trajectories",
        "movej_no_obstacle.yaml",
    ])

    arguments = [
        DeclareLaunchArgument(
            "trajectory_file",
            default_value=default_trajectory_file,
            description="Saved YAML trajectory file to execute",
        ),
        DeclareLaunchArgument(
            "start_tolerance_deg",
            default_value="5.0",
            description=(
                "Maximum allowed difference between the current joint state "
                "and the first trajectory waypoint [deg]"
            ),
        ),
        DeclareLaunchArgument(
            "execute",
            default_value="false",
            description="Execute the loaded trajectory after validation",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
    ]

    node = Node(
        package="my_arm_motion",
        executable="arm_execute_saved_exe",
        name="arm_execute_saved",
        output="screen",
        parameters=[{
            "trajectory_file": LaunchConfiguration("trajectory_file"),
            "start_tolerance_deg": LaunchConfiguration("start_tolerance_deg"),
            "execute": LaunchConfiguration("execute"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription(arguments + [node])
