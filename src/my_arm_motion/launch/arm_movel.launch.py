#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    target_xyz = DeclareLaunchArgument(
        "target_xyz",
        default_value="[0.0, -400.0, 400.0]",
        description="MoveL target position [mm] as [x,y,z]",
    )

    target_xyz_m = PythonExpression([
        "[x / 1000.0 for x in ",
        LaunchConfiguration("target_xyz"),
        "]",
    ])

    target_rpy = DeclareLaunchArgument(
        "target_rpy",
        default_value="[90.0, 0.0, 0.0]",
        description="MoveL target orientation [deg] as [roll,pitch,yaw]",
    )

    target_rpy_rad = PythonExpression([
        "[x * 3.141592653589793 / 180.0 for x in ",
        LaunchConfiguration("target_rpy"),
        "]",
    ])

    arguments = [
        target_xyz,
        target_rpy,
        DeclareLaunchArgument(
            "max_step",
            default_value="0.005",
            description="Maximum Cartesian interpolation step [m]",
        ),
        DeclareLaunchArgument(
            "fraction_threshold",
            default_value="1.0",
            description="Minimum accepted Cartesian path fraction [0,1]",
        ),
        DeclareLaunchArgument(
            "jump_threshold",
            default_value="0.0",
            description=(
                "Relative joint jump threshold; "
                "0 disables MoveIt's joint-jump check"
            ),
        ),
        DeclareLaunchArgument(
            "avoid_collisions",
            default_value="true",
            description=(
                "Enable collision checking while computing "
                "the Cartesian path"
            ),
        ),
        DeclareLaunchArgument(
            "max_velocity",
            default_value="0.2",
            description="Maximum velocity scaling factor in the range (0,1]",
        ),
        DeclareLaunchArgument(
            "max_acceleration",
            default_value="0.2",
            description=(
                "Maximum acceleration scaling factor in the range (0,1]"
            ),
        ),
        DeclareLaunchArgument(
            "motion_timeout_sec",
            default_value="180.0",
            description="Maximum planning or execution wait time [s]",
        ),
        DeclareLaunchArgument(
            "execute",
            default_value="false",
            description="Execute the trajectory after successful planning",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
    ]

    node = Node(
        package="my_arm_motion",
        executable="arm_movel_exe",
        name="arm_movel",
        output="screen",
        parameters=[{
            "group_name": "arm",
            "ik_link": "tool",
            "target_frame": "base_link",
            "planning_frame": "base_link",

            "target_xyz": target_xyz_m,
            "target_rpy": target_rpy_rad,

            "max_step": LaunchConfiguration("max_step"),
            "fraction_threshold": LaunchConfiguration(
                "fraction_threshold"
            ),
            "jump_threshold": LaunchConfiguration("jump_threshold"),
            "avoid_collisions": LaunchConfiguration(
                "avoid_collisions"
            ),

            "max_velocity": LaunchConfiguration("max_velocity"),
            "max_acceleration": LaunchConfiguration("max_acceleration"),
            "motion_timeout_sec": LaunchConfiguration(
                "motion_timeout_sec"
            ),

            "execute": LaunchConfiguration("execute"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription(arguments + [node])