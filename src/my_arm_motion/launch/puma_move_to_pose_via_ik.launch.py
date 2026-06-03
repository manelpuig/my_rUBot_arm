from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    params_file = PathJoinSubstitution([
        FindPackageShare("my_arm_motion"),
        "config",
        "puma_pose.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),

        Node(
            package="my_arm_motion",
            executable="move_to_pose",
            name="move_to_pose",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                    "planning_frame": "base_link",
                    "group_name": "arm",
                    "ik_link": "puma_tool",
                    "execute": True,
                    "max_velocity": 0.2,
                    "max_acceleration": 0.2,
                },
            ],
        ),
    ])