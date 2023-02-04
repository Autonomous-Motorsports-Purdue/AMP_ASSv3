from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    get_package_share_path('teleop_twist_joy') / 'launch' /
                    'teleop-launch.py')),
            launch_arguments={
                'config_filepath':
                str(
                    get_package_share_path('amp_launch_tests') / 'config' /
                    'xbox.config.yaml')
            }.items(),
        ),
    ])
