from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "publish_rate_hz",
            default_value="10.0",
            description="Maximum publication rate while moving the sliders",
        ),
        DeclareLaunchArgument(
            "duration",
            default_value="0.0",
            description="Trajectory point duration in seconds",
        ),
        Node(
            package="my_arm_driver",
            executable="joint_slider_gui_node",
            name="joint_slider_gui",
            output="screen",
            parameters=[{
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"), value_type=float
                ),
                "duration": ParameterValue(
                    LaunchConfiguration("duration"), value_type=float
                ),
            }],
        ),
    ])
