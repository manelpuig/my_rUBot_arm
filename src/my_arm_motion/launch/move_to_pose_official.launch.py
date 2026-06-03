from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock if Gazebo is running.",
        ),

        Node(
            package="my_arm_motion",
            executable="move_to_pose_official",
            name="move_to_pose_official",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,

                    "joint_names": [
                        "joint1",
                        "joint2",
                        "joint3",
                        "joint4",
                        "joint5",
                        "joint6",
                    ],

                    "group_name": "arm",
                    "base_link": "base_link",
                    "end_effector": "puma_tool",

                    "target_xyz": [0.40, 0.00, 0.30],
                    "target_rpy": [0.0, 3.14159, 0.0],

                    "planner_id": "RRTConnectkConfigDefault",
                    "max_velocity": 0.2,
                    "max_acceleration": 0.2,

                    "cartesian": False,
                    "cartesian_max_step": 0.0025,
                    "cartesian_fraction_threshold": 0.0,
                }
            ],
        ),
    ])