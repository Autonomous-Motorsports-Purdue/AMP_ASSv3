import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('amp_kart_segmentation')
    params_dir = os.path.join(share_dir, 'params', 'segmentation.params.yaml')

    tf_pub = Node(name="base_tf_pub",
                  package="tf2_ros",
                  executable="static_transform_publisher",
                  arguments=["0", "0", "0", "0", "0", "0", "map", "velodyne"])

    segmentation_node = Node(
        package='amp_kart_segmentation',
        executable='segmentation',
        parameters=[params_dir],
        remappings=[('~/input', '/velodyne_points'),
                    ('~/output', '/nonground')],
        name='RANSACsegmentation',
        output='screen',
        emulate_tty=True,
        #  prefix=["xterm -e gdb ex run --args"]
    )

    ld = LaunchDescription([segmentation_node, tf_pub])

    return ld
