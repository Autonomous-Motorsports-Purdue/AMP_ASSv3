import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share_dir = get_package_share_path('amp_kart_bringup')
    launch_dir = os.path.join(share_dir, 'launch')
    #kart stuff
    kart_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'kart.launch.py')),
        launch_arguments={
        }.items()
    )
    #nav2
    nav2_bringup_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'kart.launch.py')),
        launch_arguments={
        }.items()
        )
    
    ld = LaunchDescription()

    ld.addAction(kart_launch_description)
    ld.addAction(nav2_bringup_launch_description)

    return ld