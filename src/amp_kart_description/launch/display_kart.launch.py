from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def generate_launch_description():
    share_path = get_package_share_path('amp_kart_description')
    model_path = share_path / 'urdf' / 'racecar.xacro'
    default_rviz_config_path = share_path / 'rviz' / 'urdf.rviz.yaml'

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file')

    robot_description = ParameterValue(xacro.process_file(
        str(model_path)).toprettyxml(indent='  '),
                                       value_type=str)

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description':
                                          robot_description
                                      }])

    joint_state_publisher_node = Node(package='joint_state_publisher',
                                      executable='joint_state_publisher',
                                      condition=UnlessCondition(
                                          LaunchConfiguration('gui')))

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')))

    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='screen',
                     arguments=['-d', LaunchConfiguration('rvizconfig')])

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])
