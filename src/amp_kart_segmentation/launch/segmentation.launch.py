import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('amp_kart_segmentation')
    params_dir = os.path.join(share_dir, 'params', 'segmentation.params.yaml')

    segmentation_node = Node(package='amp_kart_segmentation',
                             executable='segmentation',
                             parameters=[params_dir],
                             name='RANSACsegmentation')

    ld = LaunchDescription([segmentation_node])

    return ld
