from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='serial_tty',
                              default_value='/dev/ttyACM0',
                              description='Serial TTY absolute file location'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    get_package_share_path('teleop_twist_joy') / 'launch' /
                    'teleop-launch.py'))),
        Node(package='micro_ros_agent',
             executable='micro_ros_agent',
             name='micro_ros_agent',
             arguments=['serial', '--dev',
                        LaunchConfiguration('serial_tty')])
    ])
