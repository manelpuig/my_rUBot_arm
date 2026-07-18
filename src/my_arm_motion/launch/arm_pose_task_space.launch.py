from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    target_xyz = DeclareLaunchArgument(
        "target_xyz",
        default_value="[0.0, -400.0, 300.0]",
        description="Target position [mm] as [x,y,z]",
    )
    target_xyz_m = PythonExpression([
        "[x/1000.0 for x in ",
        LaunchConfiguration("target_xyz"),
        "]",
    ])

    target_rpy = DeclareLaunchArgument(
        "target_rpy",
        default_value="[0.0, 90.0, 0.0]",
        description="Target orientation [deg] as [roll,pitch,yaw]",
    )
    target_rpy_rad = PythonExpression([
        "[x*3.141592653589793/180.0 for x in ",
        LaunchConfiguration("target_rpy"),
        "]",
    ])

    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    execute = DeclareLaunchArgument("execute", default_value="true")
    max_velocity = DeclareLaunchArgument("max_velocity", default_value="0.2")
    max_acceleration = DeclareLaunchArgument(
        "max_acceleration", default_value="0.2"
    )
    position_tolerance = DeclareLaunchArgument(
        "position_tolerance", default_value="0.005"
    )
    orientation_tolerance = DeclareLaunchArgument(
        "orientation_tolerance", default_value="0.01"
    )

    node = Node(
        package="my_arm_motion",
        executable="arm_pose_task_space_exe",
        name="arm_move_to_pose_task_space",
        output="screen",
        parameters=[{
            "group_name": "arm",
            "ik_link": "tool",
            "target_frame": "base_link",
            "planning_frame": "base_link",
            "target_xyz": target_xyz_m,
            "target_rpy": target_rpy_rad,
            "execute": LaunchConfiguration("execute"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "max_velocity": LaunchConfiguration("max_velocity"),
            "max_acceleration": LaunchConfiguration("max_acceleration"),
            "position_tolerance": LaunchConfiguration("position_tolerance"),
            "orientation_tolerance": LaunchConfiguration(
                "orientation_tolerance"
            ),
        }],
    )

    return LaunchDescription([
        target_xyz,
        target_rpy,
        use_sim_time,
        execute,
        max_velocity,
        max_acceleration,
        position_tolerance,
        orientation_tolerance,
        node,
    ])
