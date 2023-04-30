import os

from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('amp_kart_bringup')
    launch_dir = os.path.join(share_dir, 'launch')

    kart_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'kart.launch.py')))

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup.launch.py')))

    twist_mux_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'twist_mux.launch.py')))

    ld = LaunchDescription()

    ld.add_action(kart_bringup_cmd)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(twist_mux_cmd)

    return ld
