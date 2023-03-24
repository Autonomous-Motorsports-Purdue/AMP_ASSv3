import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (GroupAction, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def generate_launch_description():
    bringup_share_dir = get_package_share_directory('amp_kart_bringup')
    zed_wrapper_share_dir = get_package_share_directory('zed_wrapper')
    share_path = get_package_share_directory('amp_kart_description')
    model_path = os.path.join(share_path, 'urdf', 'racecar.xacro')

    robot_description = ParameterValue(xacro.process_file(
        str(model_path)).toprettyxml(indent='  '),
                                       value_type=str)

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description':
                                          robot_description
                                      }])

    sensor_launch_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share_dir, 'launch', 'VLP16.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zed_wrapper_share_dir, 'launch',
                             'zed.launch.py'))),
    ])

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher_node)
    ld.add_action(sensor_launch_group)

    return ld
