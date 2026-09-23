from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('livox_filter_27'),
        'config',
        'filter_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='livox_filter_27',
            executable='livox_filter_27_node',
            # 必须和 yaml 顶层的 key、以及 cpp 里的 node_name 一致
            name='threeD_lidar_filter_pointcloud',
            output='screen',
            parameters=[config_file]
        )
    ])
