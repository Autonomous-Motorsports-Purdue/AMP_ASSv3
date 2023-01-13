from launch import LaunchDescription
from launch.launch_description import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')

    return LaunchDescription([
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        Node(package='joy',
             executable='joy_node',
             name='joy_node',
             parameters=[{
                 'dev': joy_dev,
                 'deadzone': 0.3,
                 'autorepeat_rate': 20.0,
             }])
    ])
