from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    target_xyz = DeclareLaunchArgument(
        "target_xyz",
        default_value="[0.0, -400.0, 400.0]",
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

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("max_velocity", default_value="0.2"),
        DeclareLaunchArgument("max_acceleration", default_value="0.2"),
        DeclareLaunchArgument(
            "max_step",
            default_value="0.005",
            description="Maximum Cartesian interpolation step [m]",
        ),
        DeclareLaunchArgument(
            "fraction_threshold",
            default_value="1.0",
            description="Minimum accepted Cartesian path fraction; 1 requires the full MoveL",
        ),
        DeclareLaunchArgument(
            "jump_threshold",
            default_value="0.0",
            description="MoveIt relative joint-jump threshold; 0 disables it",
        ),
        DeclareLaunchArgument(
            "avoid_collisions",
            default_value="true",
            description="Check collisions while computing the Cartesian path",
        ),
        DeclareLaunchArgument("motion_timeout_sec", default_value="180.0"),
        DeclareLaunchArgument("check_singularities", default_value="true"),
        DeclareLaunchArgument("jacobian_delta", default_value="0.0001"),
        DeclareLaunchArgument("singularity_samples", default_value="20"),
        DeclareLaunchArgument("min_singular_value", default_value="0.01"),
        DeclareLaunchArgument("max_condition_number", default_value="200.0"),
        DeclareLaunchArgument("max_joint_jump_deg", default_value="45.0"),
    ]

    node = Node(
        package="my_arm_motion",
        executable="arm_movel_sing_exe",
        name="arm_movel_sing",
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
            "check_singularities": LaunchConfiguration("check_singularities"),
            "jacobian_delta": LaunchConfiguration("jacobian_delta"),
            "singularity_samples": LaunchConfiguration("singularity_samples"),
            "min_singular_value": LaunchConfiguration("min_singular_value"),
            "max_condition_number": LaunchConfiguration(
                "max_condition_number"
            ),
            "max_joint_jump_deg": LaunchConfiguration("max_joint_jump_deg"),
        }],
    )

    return LaunchDescription([
        target_xyz,
        target_rpy,
        *arguments,
        node,
    ])
