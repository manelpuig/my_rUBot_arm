from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "sequence_file",
            default_value="ur5e_movej_movel.yaml",
            description="YAML basename from config/ or an absolute YAML path",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("avoid_collisions", default_value="true"),
        DeclareLaunchArgument("max_velocity", default_value="0.1"),
        DeclareLaunchArgument("max_acceleration", default_value="0.1"),
        DeclareLaunchArgument("ik_timeout_sec", default_value="0.5"),
        DeclareLaunchArgument("joint_tolerance", default_value="0.001"),
        DeclareLaunchArgument("motion_timeout_sec", default_value="180.0"),
        DeclareLaunchArgument("check_singularities", default_value="true"),
        DeclareLaunchArgument("jacobian_delta", default_value="0.0001"),
        DeclareLaunchArgument("singularity_samples", default_value="20"),
        DeclareLaunchArgument("min_singular_value", default_value="0.01"),
        DeclareLaunchArgument("max_condition_number", default_value="200.0"),
        DeclareLaunchArgument("max_joint_jump_deg", default_value="45.0"),
    ]

    node = Node(
        package="my_arm_motion",
        executable="arm_motion_sequence_exe",
        name="arm_motion_sequence",
        output="screen",
        parameters=[{
            "group_name": "arm",
            "ik_link": "tool",
            "target_frame": "base_link",
            "planning_frame": "base_link",
            "sequence_file": LaunchConfiguration("sequence_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "execute": LaunchConfiguration("execute"),
            "avoid_collisions": LaunchConfiguration("avoid_collisions"),
            "max_velocity": LaunchConfiguration("max_velocity"),
            "max_acceleration": LaunchConfiguration("max_acceleration"),
            "ik_timeout_sec": LaunchConfiguration("ik_timeout_sec"),
            "joint_tolerance": LaunchConfiguration("joint_tolerance"),
            "motion_timeout_sec": LaunchConfiguration("motion_timeout_sec"),
            "check_singularities": LaunchConfiguration("check_singularities"),
            "jacobian_delta": LaunchConfiguration("jacobian_delta"),
            "singularity_samples": LaunchConfiguration("singularity_samples"),
            "min_singular_value": LaunchConfiguration("min_singular_value"),
            "max_condition_number": LaunchConfiguration(
                "max_condition_number"
            ),
            "max_joint_jump_deg": LaunchConfiguration("max_joint_jump_deg"),
        }],
    )

    return LaunchDescription([*arguments, node])
