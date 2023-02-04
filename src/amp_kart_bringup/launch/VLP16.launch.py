import yaml

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share_path = get_package_share_path('amp_kart_bringup')
    params_file = str(share_path / 'params' / 'VLP16.params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)

    driver_params = params['velodyne_driver_node']['ros__parameters']
    convert_params = params['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = str(share_path / 'params' / 'VLP16db.yaml')

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
                           plugin='velodyne_pointcloud::Transform',
                           name='velodyne_transform_node',
                           parameters=[convert_params]),
        ],
        output='both')

    return LaunchDescription([container])
