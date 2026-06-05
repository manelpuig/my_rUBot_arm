from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    target_xyz = DeclareLaunchArgument(
        "target_xyz",
        default_value="[0.45, 0.0, 0.35]",
        description="Target position in m",
    )

    target_rpy = DeclareLaunchArgument(
        "target_rpy",
        default_value="[0.0, 0.0, 0.0]",
        description="Target orientation in rad",
    )

    node = Node(
        package="my_arm_kinematics",
        executable="puma_ikine_pose_exe",
        name="puma_ikine_pose",
        output="screen",
        parameters=[{
            "target_xyz": LaunchConfiguration("target_xyz"),
            "target_rpy": LaunchConfiguration("target_rpy"),
        }],
    )

    return LaunchDescription([
        target_xyz,
        target_rpy,
        node,
    ])