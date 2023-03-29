import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (GroupAction, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, PythonExpression
import xacro
#import stuff for rviz2, hopefully its all we need
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from nav2_common.launch import ReplaceString


def generate_launch_description():

    #maybe change bringup_share_dir name to bringup_dir
    #to be consistent with bringup file
    bringup_share_dir = get_package_share_directory('amp_kart_bringup')
    zed_wrapper_share_dir = get_package_share_directory('zed_wrapper')
    share_path = get_package_share_directory('amp_kart_description')
    model_path = os.path.join(share_path, 'urdf', 'racecar.xacro')

    #create nav2 launch configuration variables

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    #declare nav2 launch arguments

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_share_dir, 'params',
                                   'nav2.params.yaml'),
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

    #for robot state publisher node
    robot_description = ParameterValue(xacro.process_file(
        str(model_path)).toprettyxml(indent='  '),
                                       value_type=str)

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description':
                                          robot_description
                                      }])
    #launching sensors and nav2
    sensor_nav2_launch_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share_dir, 'launch', 'VLP16.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zed_wrapper_share_dir, 'launch',
                             'zed.launch.py'))),
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
    ])

    #for rviz, there's probably a way to use the rviz launch file
    #and make this much shorter but im not sure how to do that
    #so I copied the entire thing into this launch file.

    #create rviz2 launch configuration variables
    #not sure if changing namespace to rviz_namespace works
    rviz_namespace = LaunchConfiguration('rviz_namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    rviz_config_file = LaunchConfiguration('rviz_config')

    #declare rviz launch arguments
    #for rviz namespace, depending on if default_value matters
    #can remove and use the same namespace for nav2
    #right now, I change namespace to rviz_namespace
    #added comments to namespaces later on, might have to change them
    declare_rviz_namespace_cmd = DeclareLaunchArgument(
        'rviz_namespace',
        default_value='navigation',
        description=(
            'Top-level namespace. The value will be used to replace the '
            '<robot_namespace> keyword on the rviz config file.'))

    declare_rviz_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_share_dir, 'rviz',
                                   'nav2_default_view.rviz.yaml'),
        description='Full path to the RVIZ config file to use')

    #for launching rviz

    start_rviz_cmd = Node(condition=UnlessCondition(use_namespace),
                          package='rviz2',
                          executable='rviz2',
                          name='rviz2',
                          arguments=['-d', rviz_config_file],
                          output='screen')

    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_config_file,
        #might have to change namespace to rviz_namespace?
        replacements={'<robot_namespace>': ('/', namespace)})

    start_namespaced_rviz_cmd = Node(
        condition=IfCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        #might have to change namespace to rviz_namespace?
        namespace=namespace,
        arguments=['-d', namespaced_rviz_config_file],
        output='screen',
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')])

    exit_event_handler = RegisterEventHandler(
        condition=UnlessCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    exit_event_handler_namespaced = RegisterEventHandler(
        condition=IfCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    ld = LaunchDescription()
    #add action for robot state publisher node
    ld.add_action(robot_state_publisher_node)

    #add action for nav2 launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)

    #add action for launching sensor and nav2
    ld.add_action(sensor_nav2_launch_group)

    #add action for rviz launch arguments
    ld.add_action(declare_rviz_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add any conditioned actions
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_namespaced_rviz_cmd)

    # Add other nodes and processes we need
    ld.add_action(exit_event_handler)
    ld.add_action(exit_event_handler_namespaced)

    return ld
