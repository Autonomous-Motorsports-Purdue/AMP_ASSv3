from launch import LaunchDescription
from launch.launch_description import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='serial_tty',
                              default_value='/dev/ttyACM0',
                              description='Serial TTY absolute file location'),
        Node(package='micro_ros_agent',
             executable='micro_ros_agent',
             name='micro_ros_agent',
             arguments=['serial', '--dev',
                        LaunchConfiguration('serial_tty')])
    ])
