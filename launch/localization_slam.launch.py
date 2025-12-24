#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('avular_kalman_slam')

    # When using SLAM Toolbox, we only need EKF for odom->base_link
    # SLAM Toolbox publishes map->odom, so we don't need ekf_filter_node_map
    
    # EKF for odom frame only (publishes odom->base_link)
    # This provides smooth odometry from IMU
    ekf_filter_node_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        respawn=True,
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml')],
        remappings=[
            ('/odometry/filtered', '/odom'),  # Publish to /odom for Nav2
        ]
    )

    return LaunchDescription([
        ekf_filter_node_odom,
    ])

