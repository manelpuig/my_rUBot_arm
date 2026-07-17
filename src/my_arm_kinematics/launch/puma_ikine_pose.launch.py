from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ---------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation clock",
    )

    declare_target_xyz = DeclareLaunchArgument(
        "target_xyz",
        default_value="[0.40, 0.20, 0.60]",
        description="Desired TCP position [x, y, z] in metres",
    )

    declare_target_rpy_deg = DeclareLaunchArgument(
        "target_rpy_deg",
        default_value="[0.0, 30.0, 0.0]",
        description="Desired TCP orientation [roll, pitch, yaw] in degrees",
    )

    declare_elbow = DeclareLaunchArgument(
        "elbow",
        default_value="up",
        description="Elbow configuration: up or down",
    )

    declare_wrist = DeclareLaunchArgument(
        "wrist",
        default_value="noflip",
        description="Wrist configuration: noflip or flip",
    )

    declare_tool_z = DeclareLaunchArgument(
        "tool_z",
        default_value="0.15",
        description="Distance from link6 to the TCP along the local Z axis [m]",
    )

    declare_time_sec = DeclareLaunchArgument(
        "time_sec",
        default_value="5.0",
        description="Trajectory execution time [s]",
    )

    declare_base_frame = DeclareLaunchArgument(
        "base_frame",
        default_value="base_link",
        description="Base frame used for TF verification",
    )

    declare_tcp_frame = DeclareLaunchArgument(
        "tcp_frame",
        default_value="tool",
        description="TCP frame used for TF verification",
    )

    declare_controller_topic = DeclareLaunchArgument(
        "controller_topic",
        default_value="/arm_controller/joint_trajectory",
        description="Joint trajectory controller topic",
    )

    # ---------------------------------------------------------
    # IK node
    # ---------------------------------------------------------

    puma_ikine_pose_node = Node(
        package="my_arm_kinematics",
        executable="puma_ikine_pose_exe",
        name="puma_ikine_pose",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "target_xyz": LaunchConfiguration("target_xyz"),
            "target_rpy_deg": LaunchConfiguration("target_rpy_deg"),
            "elbow": LaunchConfiguration("elbow"),
            "wrist": LaunchConfiguration("wrist"),
            "tool_z": LaunchConfiguration("tool_z"),
            "time_sec": LaunchConfiguration("time_sec"),
            "base_frame": LaunchConfiguration("base_frame"),
            "tcp_frame": LaunchConfiguration("tcp_frame"),
            "controller_topic": LaunchConfiguration("controller_topic"),
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_target_xyz,
        declare_target_rpy_deg,
        declare_elbow,
        declare_wrist,
        declare_tool_z,
        declare_time_sec,
        declare_base_frame,
        declare_tcp_frame,
        declare_controller_topic,
        puma_ikine_pose_node,
    ])