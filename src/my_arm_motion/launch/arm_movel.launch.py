from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    target_xyz = DeclareLaunchArgument(
        "target_xyz",
        default_value="[0.0, -400.0, 450.0]",
        description="MoveL target position [mm] as [x,y,z]",
    )
    target_xyz_m = PythonExpression([
        "[x/1000.0 for x in ",
        LaunchConfiguration("target_xyz"),
        "]",
    ])

    target_rpy = DeclareLaunchArgument(
        "target_rpy",
        default_value="[90.0, 0.0, 0.0]",
        description="MoveL target orientation [deg] as [roll,pitch,yaw]",
    )
    target_rpy_rad = PythonExpression([
        "[x*3.141592653589793/180.0 for x in ",
        LaunchConfiguration("target_rpy"),
        "]",
    ])

    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    execute = DeclareLaunchArgument("execute", default_value="false")
    max_velocity = DeclareLaunchArgument("max_velocity", default_value="0.2")
    max_acceleration = DeclareLaunchArgument(
        "max_acceleration", default_value="0.2"
    )
    max_step = DeclareLaunchArgument(
        "max_step",
        default_value="0.005",
        description="Maximum Cartesian interpolation step [m]",
    )
    fraction_threshold = DeclareLaunchArgument(
        "fraction_threshold",
        default_value="0.95",
        description="Minimum accepted Cartesian path fraction [0,1]",
    )
    jump_threshold = DeclareLaunchArgument(
        "jump_threshold",
        default_value="0.0",
        description="Relative joint jump threshold; 0 disables the check",
    )
    avoid_collisions = DeclareLaunchArgument(
        "avoid_collisions",
        default_value="true",
        description="Check collisions while computing the Cartesian path",
    )
    motion_timeout_sec = DeclareLaunchArgument(
        "motion_timeout_sec",
        default_value="180.0",
        description="Maximum time for planning or execution [s]",
    )

    node = Node(
        package="my_arm_motion",
        executable="arm_movel_exe",
        name="arm_movel",
        output="screen",
        parameters=[{
            "group_name": "arm",
            "ik_link": "tool",
            "target_frame": "base_link",
            "planning_frame": "base_link",
            "target_xyz": target_xyz_m,
            "target_rpy": target_rpy_rad,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "execute": LaunchConfiguration("execute"),
            "max_velocity": LaunchConfiguration("max_velocity"),
            "max_acceleration": LaunchConfiguration("max_acceleration"),
            "max_step": LaunchConfiguration("max_step"),
            "fraction_threshold": LaunchConfiguration("fraction_threshold"),
            "jump_threshold": LaunchConfiguration("jump_threshold"),
            "avoid_collisions": LaunchConfiguration("avoid_collisions"),
            "motion_timeout_sec": LaunchConfiguration("motion_timeout_sec"),
        }],
    )

    return LaunchDescription([
        target_xyz,
        target_rpy,
        use_sim_time,
        execute,
        max_velocity,
        max_acceleration,
        max_step,
        fraction_threshold,
        jump_threshold,
        avoid_collisions,
        motion_timeout_sec,
        node,
    ])
