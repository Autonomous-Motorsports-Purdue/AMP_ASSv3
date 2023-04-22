import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    share_dir = get_package_share_directory('amp_kart_bringup')
    params_dir = os.path.join(share_dir, 'params')

    mux1_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('twist_mux'), 'launch',
                         'twist_mux_launch.py')),
        launch_arguments={
            'config_locks': os.path.join(params_dir,
                                         'twist_mux_1.params.yaml'),
            'config_topics': os.path.join(params_dir,
                                          'twist_mux_1.params.yaml'),
            'cmd_vel_out': 'mux1_vel'
        }.items())

    mux2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('twist_mux'), 'launch',
                         'twist_mux_launch.py')),
        launch_arguments={
            'config_locks': os.path.join(params_dir,
                                         'twist_mux_2.params.yaml'),
            'config_topics': os.path.join(params_dir,
                                          'twist_mux_2.params.yaml'),
            'cmd_vel_out': 'cmd_vel'
        }.items())

    ld = LaunchDescription()

    ld.add_action(mux1_cmd)
    ld.add_action(mux2_cmd)

    return ld
