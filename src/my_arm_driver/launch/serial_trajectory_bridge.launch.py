from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port connected to the Arduino Nano ESP32",
    )

    baudrate_arg = DeclareLaunchArgument(
        "baudrate",
        default_value="115200",
        description="Serial baudrate",
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,

        Node(
            package="my_arm_driver",
            executable="serial_trajectory_bridge_node",
            name="serial_trajectory_bridge",
            output="screen",
            parameters=[{
                "serial_port": LaunchConfiguration("serial_port"),
                "baudrate": LaunchConfiguration("baudrate"),

                "servo_center_deg": [90, 90, 90, 90, 90, 90],
                "servo_sign": [1, 1, 1, 1, 1, 1],
                "servo_min_deg": [0, 0, 0, 0, 0, 0],
                "servo_max_deg": [180, 180, 180, 180, 180, 180],
            }],
        ),
    ])