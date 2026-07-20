#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    target_xyz = DeclareLaunchArgument(
        "target_xyz",
        default_value="[0.0, -400.0, 300.0]",
        description="Target position [mm] as [x,y,z]",
    )

    target_xyz_m = PythonExpression([
        "[x / 1000.0 for x in ",
        LaunchConfiguration("target_xyz"),
        "]",
    ])

    target_rpy = DeclareLaunchArgument(
        "target_rpy",
        default_value="[0.0, 90.0, 0.0]",
        description="Target orientation [deg] as [roll,pitch,yaw]",
    )

    target_rpy_rad = PythonExpression([
        "[x * 3.141592653589793 / 180.0 for x in ",
        LaunchConfiguration("target_rpy"),
        "]",
    ])

    seed_joints = DeclareLaunchArgument(
        "seed_joints",
        default_value="[-9.0, -90.0, -90.0, 0.0, 90.0, 0.0]",
        description="Fallback IK seed joints [deg]",
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
            description="Use the current joint state as the IK seed",
        ),
        DeclareLaunchArgument(
            "avoid_collisions",
            default_value="true",
            description="Enable collision checking during IK and planning",
        ),
        DeclareLaunchArgument(
            "joint_tolerance",
            default_value="0.001",
            description="Joint-goal planning tolerance [rad]",
        ),
        DeclareLaunchArgument(
            "max_velocity",
            default_value="0.2",
            description="Maximum velocity scaling factor in the range (0,1]",
        ),
        DeclareLaunchArgument(
            "max_acceleration",
            default_value="0.2",
            description="Maximum acceleration scaling factor in the range (0,1]",
        ),
        DeclareLaunchArgument(
            "ik_timeout_sec",
            default_value="0.5",
            description="Maximum time allowed for the IK request [s]",
        ),
        DeclareLaunchArgument(
            "motion_timeout_sec",
            default_value="180.0",
            description="Maximum planning or execution wait time [s]",
        ),
        DeclareLaunchArgument(
            "print_joints",
            default_value="true",
            description="Print the calculated joint target",
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

    arm_pose_node = Node(
        package="my_arm_motion",
        executable="arm_movej_exe",
        name="arm_move_to_pose",
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

            "avoid_collisions": LaunchConfiguration("avoid_collisions"),
            "joint_tolerance": LaunchConfiguration("joint_tolerance"),
            "max_velocity": LaunchConfiguration("max_velocity"),
            "max_acceleration": LaunchConfiguration("max_acceleration"),
            "ik_timeout_sec": LaunchConfiguration("ik_timeout_sec"),
            "motion_timeout_sec": LaunchConfiguration(
                "motion_timeout_sec"
            ),
            "print_joints": LaunchConfiguration("print_joints"),
            "execute": LaunchConfiguration("execute"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription(arguments + [arm_pose_node])