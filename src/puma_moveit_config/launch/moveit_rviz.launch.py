from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("my_arm", package_name="puma_moveit_config")
        .robot_description(file_path="config/my_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/my_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    use_sim_time = {"use_sim_time": True}

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            use_sim_time,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[],
        parameters=[
            moveit_config.to_dict(),
            use_sim_time,
        ],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])