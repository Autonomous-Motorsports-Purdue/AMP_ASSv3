import os
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_share_path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro

def vlp16():
    share_path = get_package_share_path('amp_kart_bringup')
    params_file = str(share_path / 'params' / 'VLP16.params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)

    driver_params = params['velodyne_driver_node']['ros__parameters']
    convert_params = params['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = str(share_path / 'params' / 'VLP16db.yaml')
    laserscan_params = params['velodyne_laserscan_node']['ros__parameters']

    container = ComposableNodeContainer(
        name='velodyne_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(package='velodyne_driver',
                           plugin='velodyne_driver::VelodyneDriver',
                           name='velodyne_driver_node',
                           parameters=[driver_params]),
            ComposableNode(package='velodyne_pointcloud',
                           plugin='velodyne_pointcloud::Convert',
                           name='velodyne_convert_node',
                           parameters=[convert_params]),
            ComposableNode(package='velodyne_laserscan',
                           plugin='velodyne_laserscan::VelodyneLaserScan',
                           name='velodyne_laserscan_node',
                           parameters=[laserscan_params]),
        ],
        output='both')
        
    return container
   
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
                                          robot_description,
                                          'use_tf_static': 'false',
                                          'ignore_timestamp': 'false'
                                      }])

    sensor_launch_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share_dir, 'launch', 'VLP16.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share_dir, 'launch', 'zed.launch.py'))),
    ])

    patchworkpp_demo_node = Node(package='patchworkpp',
                                 executable='demo',
                                 remappings=[('cloud', 'patchworkpp_cloud'),
-                                             ('ground', 'patchworkpp_ground'),
-                                             ('nonground',
-                                              'patchworkpp_nonground')],
                                 parameters=[
                                     os.path.join(bringup_share_dir, 'params',
                                                  'patchworkpp.params.yaml')
                                 ])

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[('scan', 'flattened_pointcloud'),
                    ('cloud_in', 'velodyne_points')])

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(name='serial_tty',
                              default_value='/dev/ttyUSB0',
                              description='Serial TTY absolute file location'))

    ld.add_action(micro_ros_agent_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(sensor_launch_group)
    ld.add_action(patchworkpp_demo_node)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(vlp16())

    return ld
