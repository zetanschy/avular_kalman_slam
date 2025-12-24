#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('avular_kalman_slam')

    # Launch argument to choose filter type
    filter_type_arg = DeclareLaunchArgument(
        'filter_type',
        default_value='ukf',  # Default to UKF
        description='Filter type: ukf, ekf, or dual_ekf'
    )

    filter_type = LaunchConfiguration('filter_type')

    # Static transform from map to odom (initial offset)
    # This will be updated by navsat_transform_node when GPS data is received
    map_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_transform',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0', 
                   '--roll', '0', '--pitch', '0', '--yaw', '0', 
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    # UKF Node (for filter_type='ukf')
    ukf_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_node',
        output='screen',
        respawn=True,
        parameters=[os.path.join(pkg_share, 'config/ukf.yaml')],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ],
        condition=IfCondition(PythonExpression(["'", filter_type, "' == 'ukf'"]))
    )

    # Single EKF Node (for filter_type='ekf')
    ekf_filter_node_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        respawn=True,
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml')],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ],
        condition=IfCondition(PythonExpression(["'", filter_type, "' == 'ekf'"]))
    )

    # Dual EKF - odom frame (for filter_type='dual_ekf')
    ekf_filter_node_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        respawn=True,
        parameters=[os.path.join(pkg_share, 'config/dual_ekf.yaml')],
        remappings=[
            ('/odometry/filtered', '/odometry/local'),
        ],
        condition=IfCondition(PythonExpression(["'", filter_type, "' == 'dual_ekf'"]))
    )

    # Dual EKF - map frame (for filter_type='dual_ekf')
    ekf_filter_node_map_dual = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        respawn=True,
        parameters=[os.path.join(pkg_share, 'config/dual_ekf.yaml')],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ],
        condition=IfCondition(PythonExpression(["'", filter_type, "' == 'dual_ekf'"]))
    )

    # NavSat Transform Node: Converts GPS (NavSatFix) to odometry
    # Config depends on filter type
    navsat_transform_node_ukf = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{
            "magnetic_declination_radians": 0.0,
            "yaw_offset": 0.0,
            "zero_altitude": True,
            "use_odometry_yaw": False,
            "wait_for_datum": False,
            "publish_filtered_gps": False,
            "broadcast_utm_transform": False,
            "use_simtime": True,
        }],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ],
        condition=IfCondition(PythonExpression(["'", filter_type, "' == 'ukf'"]))
    )

    navsat_transform_node_ekf = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml')],
        remappings=[
            ('/imu/data', '/imu'),
            ('/gps/fix', '/gps/fix'),
            ('/odometry/gps', '/odometry/gps'),
        ],
        condition=IfCondition(PythonExpression(["'", filter_type, "' == 'ekf'"]))
    )

    navsat_transform_node_dual = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/dual_ekf.yaml')],
        remappings=[
            ('/imu/data', '/imu'),
            ('/gps/fix', '/gps/fix'),
            ('/odometry/filtered', '/odometry/local'),  # Use odom EKF for yaw
            ('/odometry/gps', '/odometry/gps'),
        ],
        condition=IfCondition(PythonExpression(["'", filter_type, "' == 'dual_ekf'"]))
    )

    return LaunchDescription([
        filter_type_arg,
        map_transform_node,
        ukf_node,
        ekf_filter_node_map,
        ekf_filter_node_odom,
        ekf_filter_node_map_dual,
        navsat_transform_node_ukf,
        navsat_transform_node_ekf,
        navsat_transform_node_dual,
    ])
