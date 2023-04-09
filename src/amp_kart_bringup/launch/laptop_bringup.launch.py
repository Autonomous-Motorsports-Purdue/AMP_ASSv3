import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generateLaunchDescription():
    share_dir = get_package_share_path('amp_kart_bringup')
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

    #rviz
    rviz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz.launch.py')),
                                        condition=IfCondition(use_rviz),
                                        launch_arguments={
                                            'namespace': '',
                                            'use_namespace': 'False',
                                            'rviz_config': rviz_config_file
                                        }.items()
    )

    ld = LaunchDescription()
    ld.addAction(declare_use_rviz_cmd)
    ld.addAction(declare_rviz_config_file_cmd)
    ld.addAction(rviz_launch_description)

    return ld
