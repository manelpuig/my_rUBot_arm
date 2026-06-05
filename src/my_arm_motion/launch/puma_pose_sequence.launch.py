from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config_dir = os.path.join(
        get_package_share_directory("my_arm_motion"),
        "config"
    )

    sequence_file = DeclareLaunchArgument(
        "sequence_file",
        default_value="puma_give5.yaml",
        description="YAML sequence file inside config/",
    )

    sequence_file_path = PathJoinSubstitution([
        config_dir,
        LaunchConfiguration("sequence_file"),
    ])

    controller_action = DeclareLaunchArgument(
        "controller_action",
        default_value="/arm_controller/follow_joint_trajectory",
        description="FollowJointTrajectory action server",
    )

    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
    )

    puma_pose_sequence_node = Node(
        package="my_arm_motion",
        executable="puma_pose_sequence_exe",
        name="puma_pose_sequence",
        output="screen",
        parameters=[{
            "sequence_file": sequence_file_path,
            "controller_action": LaunchConfiguration("controller_action"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription([
        sequence_file,
        controller_action,
        use_sim_time,
        puma_pose_sequence_node,
    ])