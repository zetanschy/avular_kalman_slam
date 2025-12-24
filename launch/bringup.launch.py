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

    filter_type_arg = DeclareLaunchArgument(
        'filter_type',
        default_value='dual_ekf',
        description='Filter type: ukf, ekf, or dual_ekf'
    )

    # Include simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch/simulation.launch.py')
        )
    )

    # Include localization (GPS + IMU fusion)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch/localization.launch.py')
        ),
        launch_arguments={
            'filter_type': LaunchConfiguration('filter_type')
        }.items()
    )

    # Include navigation (Nav2)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch/navigation.launch.py')
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
        filter_type_arg,
        simulation,
        localization,
        navigation,
        visualization,
    ])

