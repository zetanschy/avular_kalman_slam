#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('avular_kalman_slam')

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/slam.yaml')],
        remappings=[
            ('/scan', '/scan'),
            ('/map', '/map'),
            ('/map_metadata', '/map_metadata'),
        ]
    )

    return LaunchDescription([
        slam_toolbox_node,
    ])

