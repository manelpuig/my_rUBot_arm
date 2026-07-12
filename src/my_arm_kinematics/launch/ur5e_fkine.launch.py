from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    joints = DeclareLaunchArgument(
        "joints",
        default_value="[0.0, -60.0, -135.0, -30.0, 90.0, 0.0]",
        description="UR5e joint values in degrees",
    )

    node = Node(
        package="my_arm_kinematics",
        executable="ur5e_fkine_exe",
        name="ur5e_fkine",
        output="screen",
        parameters=[{
            "joints": LaunchConfiguration("joints"),
        }],
    )

    return LaunchDescription([
        joints,
        node,
    ])