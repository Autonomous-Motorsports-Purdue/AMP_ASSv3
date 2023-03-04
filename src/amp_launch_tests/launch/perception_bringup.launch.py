import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (GroupAction, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_share_dir = get_package_share_directory('amp_kart_bringup')
    zed_wrapper_share_dir = get_package_share_directory('zed_wrapper')

    bringup_cmd_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share_dir, 'launch', 'VLP16.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zed_wrapper_share_dir, 'launch',
                             'zed.launch.py'))),
    ])

    ld = LaunchDescription()
    ld.add_action(bringup_cmd_group)

    return ld
