import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    params_file = os.path.join(get_package_share_directory('amp_kart_bringup'),
                               'params', 'switch_mux.params.yaml')

    switch_mux_action = Node(package='amp_kart_commander',
                             namespace='',
                             executable='switch_mux',
                             name='switch_mux',
                             parameters=[params_file])

    kart_commander_action = Node(package='amp_kart_commander',
                                 namespace='',
                                 executable='kart_commander',
                                 name='kart_commander')

    ld = LaunchDescription()
    ld.add_action(switch_mux_action)
    ld.add_action(kart_commander_action)

    return ld
