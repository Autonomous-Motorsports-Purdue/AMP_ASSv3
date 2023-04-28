import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def generate_launch_description():
    bringup_share_dir = get_package_share_directory('amp_kart_bringup')
    description_share_path = get_package_share_directory(
        'amp_kart_description')
    model_path = os.path.join(description_share_path, 'urdf', 'racecar.xacro')

    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev',
                   LaunchConfiguration('serial_tty')])

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
                os.path.join(bringup_share_dir, 'launch', 'zed.launch.py'))),
    ])

    pointcloud_to_laserscan_node = Node(package='pointcloud_to_laserscan',
                                        executable='pointcloud_to_laserscan',
                                        remappings=[
                                            ('scan', 'flattened_pointcloud'),
                                            ('cloud_in', 'velodyne_points')
                                        ])

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(name='serial_tty',
                              default_value='/dev/ttyUSB0',
                              description='Serial TTY absolute file location'))

    ld.add_action(micro_ros_agent_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(sensor_launch_group)

    return ld
