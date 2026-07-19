#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    box_xyz = DeclareLaunchArgument(
        "box_xyz",
        default_value="[0.0,-200.0,400.0]",
        description="Box centre [mm] as [x,y,z]",
    )
    box_xyz_m = PythonExpression([
        "[x/1000.0 for x in ",
        LaunchConfiguration("box_xyz"),
        "]",
    ])

    box_size = DeclareLaunchArgument(
        "box_size",
        default_value="[180.0,180.0,550.0]",
        description="Box dimensions [mm] as [x,y,z]",
    )
    box_size_m = PythonExpression([
        "[x/1000.0 for x in ",
        LaunchConfiguration("box_size"),
        "]",
    ])

    arguments = [
        DeclareLaunchArgument("operation", default_value="add"),
        DeclareLaunchArgument("object_id", default_value="moveit_test_box"),
        DeclareLaunchArgument("frame_id", default_value="base_link"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]

    node = Node(
        package="my_arm_motion",
        executable="arm_test_scene_exe",
        name="arm_test_scene",
        output="screen",
        parameters=[{
            "operation": LaunchConfiguration("operation"),
            "object_id": LaunchConfiguration("object_id"),
            "frame_id": LaunchConfiguration("frame_id"),
            "box_xyz": box_xyz_m,
            "box_size": box_size_m,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription([
        box_xyz,
        box_size,
        *arguments,
        node,
    ])
