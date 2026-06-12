from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial port connected to the Arduino Nano ESP32",
        ),

        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="Serial baudrate",
        ),

        Node(
            package="my_arm_driver",
            executable="serial_bridge_node",
            name="serial_bridge_node",
            output="screen",
            parameters=[{
                "serial_port": LaunchConfiguration("serial_port"),
                "baudrate": LaunchConfiguration("baudrate"),

                "servo_center_deg": [90, 90, 90, 90, 90, 90],
                "servo_sign": [1, 1, 1, 1, 1, 1],
            }],
        ),
    ])