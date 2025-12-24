#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('avular_kalman_slam')

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Launch RViz visualization'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='slam_world.world',
        description='World file to load'
    )

    # Include simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch/simulation.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )

    # Include localization for SLAM (only EKF odom, no map frame)
    # SLAM Toolbox will publish map->odom, EKF only publishes odom->base_link
    localization_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch/localization_slam.launch.py')
        )
    )

    # Include SLAM Toolbox
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch/slam.launch.py')
        )
    )

    # Include visualization (RViz)
    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch/visualization.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        world_arg,
        simulation,
        localization_slam,
        slam,
        visualization,
    ])

