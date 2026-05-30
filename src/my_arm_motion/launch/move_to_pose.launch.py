from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motion_config = LaunchConfiguration("motion_config")
    pose_config = LaunchConfiguration("pose_config")

    declare_motion_config = DeclareLaunchArgument(
        "motion_config",
        default_value=PathJoinSubstitution([
            FindPackageShare("my_arm_motion"),
            "config",
            "puma_motion.yaml",
        ]),
        description="Robot motion configuration YAML file",
    )

    declare_pose_config = DeclareLaunchArgument(
        "pose_config",
        default_value=PathJoinSubstitution([
            FindPackageShare("my_arm_motion"),
            "config",
            "puma_pose.yaml",
        ]),
        description="Target pose YAML file",
    )

    move_to_pose_node = Node(
        package="my_arm_motion",
        executable="move_to_pose",
        name="move_to_pose",
        output="screen",
        parameters=[
            motion_config,
            pose_config,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        declare_motion_config,
        declare_pose_config,
        move_to_pose_node,
    ])