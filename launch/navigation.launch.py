#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_share = get_package_share_directory('avular_kalman_slam')

    # Nav2 parameters file (no map needed for GPS outdoor navigation)
    params_file = os.path.join(pkg_share, 'config/nav2_params.yaml')

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Start navigation (map_server removed for GPS outdoor navigation)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch/navigation_launch.py')),
        launch_arguments={'use_sim_time': 'True', 'params_file': params_file}.items(),
    )

    return LaunchDescription(
        [
            nav2_bringup_launch
        ]
    )

