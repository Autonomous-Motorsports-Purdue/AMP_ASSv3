import os

from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    share_dir = get_package_share_directory('amp_kart_bringup')
    launch_dir = os.path.join(share_dir, 'launch')

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RVIZ')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(share_dir, 'rviz',
                                   'nav2_default_view.rviz.yaml'),
        description='Full path to the RVIZ config file to use')

    rviz_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(launch_dir, 'rviz.launch.py')),
                                        condition=IfCondition(use_rviz),
                                        launch_arguments={
                                            'namespace': '',
                                            'use_namespace': 'False',
                                            'rviz_config': rviz_config_file
                                        }.items())

    teleop_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(share_dir, 'launch', 'teleop.launch.py')),
                                          launch_arguments={
                                              'config_filepath':
                                              os.path.join(
                                                  share_dir, 'config',
                                                  'xbox.config.yaml')
                                          }.items())

    ld = LaunchDescription()

    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(teleop_cmd)
    return ld
