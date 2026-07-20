#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    target_xyz = DeclareLaunchArgument(
        "target_xyz",
        default_value="[0.0, -400.0, 500.0]",
        description="Target position [mm] as [x,y,z]",
    )
    target_xyz_m = PythonExpression([
        "[x / 1000.0 for x in ",
        LaunchConfiguration("target_xyz"),
        "]",
    ])

    target_rpy = DeclareLaunchArgument(
        "target_rpy",
        default_value="[90.0, 0.0, 0.0]",
        description="Target orientation [deg] as [roll,pitch,yaw]",
    )
    target_rpy_rad = PythonExpression([
        "[x * 3.141592653589793 / 180.0 for x in ",
        LaunchConfiguration("target_rpy"),
        "]",
    ])

    seed_joints = DeclareLaunchArgument(
        "seed_joints",
        default_value="[-60.0, -60.0, -100.0, 170.0, -90.0, 0.0]",
        description="Fallback numerical IK seed [deg]",
    )
    seed_joints_rad = PythonExpression([
        "[x * 3.141592653589793 / 180.0 for x in ",
        LaunchConfiguration("seed_joints"),
        "]",
    ])

    arguments = [
        target_xyz,
        target_rpy,
        seed_joints,
        DeclareLaunchArgument(
            "seed_from_joint_states",
            default_value="true",
            description="Use the current joints as the numerical IK seed",
        ),
        DeclareLaunchArgument(
            "ik_timeout_sec",
            default_value="1.0",
            description="Maximum numerical IK search time [s]",
        ),
        DeclareLaunchArgument(
            "controller_action",
            default_value="/arm_controller/follow_joint_trajectory",
            description="Direct joint-trajectory controller action",
        ),
        DeclareLaunchArgument(
            "duration_sec",
            default_value="4.0",
            description="Direct joint movement duration [s]",
        ),
        DeclareLaunchArgument(
            "print_joints",
            default_value="true",
            description="Print the numerical IK solution",
        ),
        DeclareLaunchArgument(
            "execute",
            default_value="false",
            description="Send the IK result directly to the controller",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
    ]

    node = Node(
        package="my_arm_motion",
        executable="arm_pose_numeric_ik_exe",
        name="arm_pose_numeric_ik",
        output="screen",
        parameters=[{
            "group_name": "arm",
            "ik_link": "tool",
            "target_frame": "base_link",
            "planning_frame": "base_link",
            "target_xyz": target_xyz_m,
            "target_rpy": target_rpy_rad,
            "seed_joints": seed_joints_rad,
            "seed_from_joint_states": LaunchConfiguration(
                "seed_from_joint_states"
            ),
            "ik_timeout_sec": LaunchConfiguration("ik_timeout_sec"),
            "controller_action": LaunchConfiguration(
                "controller_action"
            ),
            "duration_sec": LaunchConfiguration("duration_sec"),
            "print_joints": LaunchConfiguration("print_joints"),
            "execute": LaunchConfiguration("execute"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription(arguments + [node])
