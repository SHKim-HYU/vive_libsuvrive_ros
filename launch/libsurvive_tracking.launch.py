"""Steam-free VIVE tracking via libsurvive (ROS2).

Usage:  ros2 launch vive_libsurvive_ros libsurvive_tracking.launch.py
Override the config:  ... config:=/abs/path/to/trackers.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vive_libsurvive_ros')
    default_config = os.path.join(pkg, 'config', 'trackers.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('node_name', default_value='vive_libsurvive_ros'),
        DeclareLaunchArgument('output', default_value='screen'),
        Node(
            package='vive_libsurvive_ros',
            executable='survive_tracking_node',
            name=LaunchConfiguration('node_name'),
            # Galactic Node(output=...) takes a literal only (substitutions land in Humble).
            output='screen',
            parameters=[{'config': LaunchConfiguration('config')}],
        ),
    ])
