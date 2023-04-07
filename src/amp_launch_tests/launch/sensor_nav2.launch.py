import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (GroupAction, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
import xacro
#import stuff for rviz2, hopefully its all we need
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    #test_dir is used for the modified kart file in launch_tests
    test_dir = get_package_share_directory('amp_launch_tests')
    share_dir = get_package_share_directory('amp_kart_bringup')
    test_launch_dir = os.path.join(test_dir, 'launch')
    launch_dir = os.path.join(share_dir, 'launch')
    #for robot description
    share_path = get_package_share_directory('amp_kart_description')
    model_path = os.path.join(share_path, 'urdf', 'racecar.xacro')
    #argument for robot state publisher node in kart file
    robot_description = LaunchConfiguration('robot_description')
    declare_robot_description_cmd = DeclareLaunchArgument(
            'robot_description',
            default_value = ParameterValue(xacro.process_file(
            str(model_path)).toprettyxml(indent='  '),
                                       value_type=str),
            description = 'Argument for robot description in kart file')
    #nav2 arguments
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'nav2.params.yaml'),
        description=
        'Full path to the ROS2 parameters file to use for all launched nodes')
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'), 'behavior_trees',
            'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')
    bringup_cmd_group = GroupAction ([
        #launch kart file, which has sensors and robot state publisher node
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(test_launch_dir, 'kart.launch.py')),
            launch_arguments = {
                'robot_description': robot_description
            }.items()
        ),
        PushRosNamespace(condition=IfCondition(use_namespace),
                         namespace=namespace),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'navigation.launch.py')),
                                 launch_arguments={
                                     'namespace': namespace,
                                     'use_sim_time': use_sim_time,
                                     'autostart': autostart,
                                     'params_file': params_file,
                                     'default_bt_xml_filename':
                                     default_bt_xml_filename,
                                     'use_lifecycle_mgr': 'false',
                                     'map_subscribe_transient_local': 'true'
                                 }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(test_launch_dir, 'rviz.launch.py')))
    ])
    ld = LaunchDescription()
    #declare robot description arg
    ld.add_action(declare_robot_description_cmd)
    #declare nav2 launch args
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    #launch sensors, nav2, and rviz
    ld.add_action(bringup_cmd_group)
    return ld
