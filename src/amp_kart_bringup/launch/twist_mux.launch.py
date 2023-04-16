import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('amp_kart_bringup')
    params_dir = os.path.join(share_dir, 'params')

    mux_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('twist_mux'), 'launch',
                         'twist_mux_launch.py')),
        launch_arguments={
            'config_locks': os.path.join(params_dir, 'twist_mux.params.yaml'),
            'config_topics': os.path.join(params_dir, 'twist_mux.params.yaml'),
            'cmd_vel_out': 'cmd_vel'
        }.items())

    kart_commander_action = Node(package='amp_kart_commander',
                                 namespace='',
                                 executable='kart_commander',
                                 name='sikart_commander')

    ld = LaunchDescription()
    ld.add_action(mux_cmd)
    ld.add_action(kart_commander_action)

    return ld
