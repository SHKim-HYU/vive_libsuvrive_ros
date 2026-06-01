"""libsurvive tracking that mimics vive_tracking_ros's topics/frames/names (ROS2).

Usage:  ros2 launch vive_libsurvive_ros vive_compat.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vive_libsurvive_ros')
    default_config = os.path.join(pkg, 'config', 'vive_compat.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('node_name', default_value='vive_libsurvive_ros'),
        DeclareLaunchArgument('output', default_value='screen'),
        Node(
            package='vive_libsurvive_ros',
            executable='survive_tracking_node',
            name=LaunchConfiguration('node_name'),
            output=LaunchConfiguration('output'),
            parameters=[{'config': LaunchConfiguration('config')}],
        ),
    ])
