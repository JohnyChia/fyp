import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('durian_inspection_pkg')
    config = os.path.join(pkg_dir, 'config', 'collector_params.yaml')

    return LaunchDescription([
        Node(
            package='durian_inspection_pkg',
            executable='collector',
            name='gazebo_collector',
            parameters=[config],
            output='screen'
        )
    ])
