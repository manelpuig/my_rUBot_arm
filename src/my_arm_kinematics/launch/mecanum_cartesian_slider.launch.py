#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "elbow",
            default_value="up",
            description="Initial analytical IK branch: up or down",
        ),
        DeclareLaunchArgument(
            "publish_rate_hz",
            default_value="10.0",
            description="Maximum publication rate while sliders move",
        ),
        DeclareLaunchArgument(
            "duration",
            default_value="0.0",
            description="Trajectory-point duration for the serial bridge",
        ),
        DeclareLaunchArgument(
            "live_send_on_start",
            default_value="true",
            description="Enable live sending when the GUI starts",
        ),
        DeclareLaunchArgument(
            "gripper_open_joint_deg",
            default_value="0.0",
            description="Calibrated joint6 angle for an open gripper",
        ),
        DeclareLaunchArgument(
            "gripper_closed_joint_deg",
            default_value="45.0",
            description="Calibrated joint6 angle for a closed gripper",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the simulation clock",
        ),
    ]

    node = Node(
        package="my_arm_kinematics",
        executable="mecanum_cartesian_slider_exe",
        name="mecanum_cartesian_slider",
        output="screen",
        parameters=[{
            "elbow": LaunchConfiguration("elbow"),
            "publish_rate_hz": ParameterValue(
                LaunchConfiguration("publish_rate_hz"), value_type=float
            ),
            "duration": ParameterValue(
                LaunchConfiguration("duration"), value_type=float
            ),
            "live_send_on_start": ParameterValue(
                LaunchConfiguration("live_send_on_start"), value_type=bool
            ),
            "gripper_open_joint_deg": ParameterValue(
                LaunchConfiguration("gripper_open_joint_deg"),
                value_type=float,
            ),
            "gripper_closed_joint_deg": ParameterValue(
                LaunchConfiguration("gripper_closed_joint_deg"),
                value_type=float,
            ),
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
        }],
    )

    return LaunchDescription(arguments + [node])
