from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.launch_description import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_filepath = LaunchConfiguration('config_filepath')

    return LaunchDescription([
        DeclareLaunchArgument(name='config_filepath',
                              default_value=str(
                                  get_package_share_path('amp_launch_tests') /
                                  'config' / 'xbox.config.yaml')),
        DeclareLaunchArgument(name='serial_tty',
                              default_value='/dev/ttyACM0',
                              description='Serial TTY absolute file location'),
        Node(package='teleop_twist_joy',
             executable='teleop_node',
             name='teleop_twist_joy_node',
             parameters=[config_filepath]),
        Node(package='micro_ros_agent',
             executable='micro_ros_agent',
             name='micro_ros_agent',
             arguments=['serial', '--dev',
                        LaunchConfiguration('serial_tty')])
    ])
