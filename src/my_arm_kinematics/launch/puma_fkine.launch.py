from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    joints = DeclareLaunchArgument(
        "joints",
        default_value="[0.0, -40.0, 70.0, 0.0, 40.0, 0.0]",
        description="Joint values in degrees",
    )

    node = Node(
        package="my_arm_kinematics",
        executable="puma_fkine_exe",
        name="puma_fkine",
        output="screen",
        parameters=[{
            "joints": LaunchConfiguration("joints"),
        }],
    )

    return LaunchDescription([
        joints,
        node,
    ])