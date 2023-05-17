import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue

import xacro
import yaml
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    bringup_share_dir = get_package_share_directory('amp_kart_bringup')

    costmap_params = os.path.join(bringup_share_dir, 'params','costmap.params.yaml')
    # costmap_params = RewrittenYaml(
    #     source_file=costmap_params,
    #     root_key='costmap',
    #     param_rewrites={},
    #     convert_types=True)

    # with open(costmap_params, 'r') as f:
    #     configuration = yaml.safe_load(f)
    #     print(f'Loaded configuration: {configuration["costmap"]["ros__parameters"]}')

    action = GroupAction(actions=[
        LifecycleNode(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='nodename',
            namespace='',
            parameters=[costmap_params],
            ),

        LifecycleNode(
            package = 'nav2_lifecycle_manager',
            executable = 'lifecycle_manager',
            name = 'lifecycle_manager',
            namespace = 'costmap_namespace',
            parameters = [
                {'autostart' : True},
                {'node_names' : ['nodename']},
            ],
        )
    ])
    
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev',
                   LaunchConfiguration('serial_tty')])

    urdf_model_path = os.path.join(get_package_share_directory('amp_kart_description'), 'urdf', 'racecar.xacro')
    robot_description = ParameterValue(xacro.process_file(
        str(urdf_model_path)).toprettyxml(indent='  '),
                                       value_type=str)

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description':
                                          robot_description
                                      }])
    
    flatten = Node(package='amp_kart_segmentation',
                    executable='segmentation',
                    name='flatten',
                    parameters=[{'resolution': 0.05},
                                {'minx': -2.},
                                {'maxx': 2.},
                                {'miny': -2.},
                                {'maxy': 2.}],
                    remappings=[
                        ('~/input', '/nonground'),
                        ('~/output', '/costmap')
                    ],
                    # prefix=['xterm -e gdb -ex run --args'],
                    )
    
    local_goal = Node(package='amp_local_goal',
                    executable='local_goal_node',
                    name='local_goal_node',
                    parameters=[os.path.join(bringup_share_dir, 'params', 'local_goal.params.yaml')],
                    remappings=[
                    ])

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
                                 remappings=[('cloud_topic', '/velodyne_points'),
                                             ('cloud', '/patchworkpp_cloud'),
                                             ('ground', '/ground'),
                                             ('nonground', '/nonground')],
                                 parameters=[
                                     os.path.join(bringup_share_dir, 'params',
                                                  'patchworkpp.params.yaml')
                                 ])

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(name='serial_tty',
                              default_value='/dev/ttyUSB0',
                              description='Serial TTY absolute file location'))

    # ld.add_action(micro_ros_agent_node)
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(sensor_launch_group)
    ld.add_action(patchworkpp_demo_node)
    ld.add_action(local_goal)
    ld.add_action(flatten)

    return ld
