#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_world_path(context):
    """Function to generate the full path to the world file from avular_kalman_slam"""
    pkg_avular_kalman = get_package_share_directory('avular_kalman_slam')
    world_name = LaunchConfiguration('world').perform(context)
    world_path = os.path.join(pkg_avular_kalman, 'world', world_name)
    # Set the world path as an absolute path that will be used instead of the world name
    return [SetLaunchConfiguration('world_absolute_path', world_path)]


def generate_launch_description():
    # Get package directories
    pkg_avular_sim = get_package_share_directory('origin_one_gazebo')
    pkg_avular_kalman = get_package_share_directory('avular_kalman_slam')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.world',  # Default to slam_world.world from avular_kalman_slam
        description='World file to load from avular_kalman_slam/world/'
    )

    drive_config_arg = DeclareLaunchArgument(
        'drive_configuration',
        default_value='skid_steer_drive',
        description='Drive configuration: skid_steer_drive or mecanum_drive'
    )

    use_cmd_vel_controller_arg = DeclareLaunchArgument(
        'use_cmd_vel_controller',
        default_value='False',
        description='Use cmd_vel_controller package'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )

    # Generate world path from avular_kalman_slam
    world_path_function = OpaqueFunction(function=generate_world_path)

    # Launch Gazebo with world from avular_kalman_slam
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v1 ', LaunchConfiguration('world_absolute_path')],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Launch Gazebo GUI
    gz_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g', '-v1'],
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )

    # Now we need to spawn the robot and set up bridges
    # We'll do this manually instead of using origin_sim_common to avoid loading the world twice
    pkg_dir_origin_description = get_package_share_directory('origin_one_description')
    xacro_file_path = 'urdf/origin_one.urdf.xacro'
    
    # Set up environment for models
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = (
        os.path.join(pkg_avular_sim, 'models')
        + os.pathsep
        + os.path.join(pkg_dir_origin_description, '..')
    )

    # Function to create the URDF description from the Xacro file
    def create_urdf_description(context):
        xacro_file = os.path.join(
            FindPackageShare('origin_one_description').find('origin_one_description'),
            xacro_file_path
        )
        drive_configuration = LaunchConfiguration('drive_configuration').perform(context)
        urdf = xacro.process_file(
            xacro_file,
            mappings={'drive_configuration': drive_configuration}
        ).toxml()
        return [SetLaunchConfiguration('urdf', urdf)]

    create_urdf_action = OpaqueFunction(function=create_urdf_description)

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'Robot',
            '-string', LaunchConfiguration('urdf'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
            '-Y', '0.0',
        ],
        output='screen',
    )

    # Bridge configuration
    bridge_config_file = PathJoinSubstitution([
        pkg_avular_sim,
        'config',
        'gazebo_ros_bridge_topics.yaml',
    ])

    # ROS-Gazebo bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config_file,
            'qos_overrides./robot/cmd_vel.subscription.reliability': 'best_effort',
        }],
    )

    # Include robot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir_origin_description, 'launch', 'origin_one_description.launch.py')
        )
    )

    # Robot specifics node
    robot_specifics = Node(
        package='origin_one_gazebo',
        executable='robot_specifics',
        arguments=[],
        output='screen',
    )

    # TF static transforms for camera
    camera_color_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link', 'camera_color_frame'],
        output='screen',
    )

    camera_color_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '-1.5707', '0.0', '-1.5707', 'camera_color_frame', 'camera_color_optical_frame'],
        output='screen',
    )

    camera_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link', 'camera_depth_frame'],
        output='screen',
    )

    camera_depth_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '-1.5707', '0.0', '-1.5707', 'camera_color_frame', 'camera_depth_optical_frame'],
        output='screen',
    )

    os_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.006253', '-0.011775', '0.007645', '0.0', '0.0', '0.0', 'os_sensor', 'os_imu'],
        output='screen',
    )

    os_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.03618', '0.0', '0.0', '0.0', 'os_sensor', 'os_lidar'],
        output='screen',
    )

    # cmd_vel_controller
    cmd_vel_controller = Node(
        condition=IfCondition(LaunchConfiguration('use_cmd_vel_controller')),
        package='cmd_vel_controller',
        executable='cmd_vel_controller',
        name='cmd_vel_controller',
        namespace='robot',
        parameters=[PathJoinSubstitution([pkg_dir_origin_description, 'config', 'cmd_vel_controller.yaml'])],
        respawn=True,
        respawn_delay=1.0,
    )

    # Remap GPS from Avular format to standard name
    # Avular bridges GPS to /robot/gnss/fix_off, but navsat_transform expects /gps/fix
    gps_relay = Node(
        package='topic_tools',
        executable='relay',
        name='gps_relay',
        arguments=['/robot/gnss/fix_off', '/gps/fix'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Bridge IMU from Gazebo
    # IMU sensor publishes to /imu topic in Gazebo (from the sensor configuration)
    # We need to bridge it to ROS topic /imu
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Relay cmd_vel from Nav2 (/cmd_vel) to Avular Origin (/robot/cmd_vel)
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel', '/robot/cmd_vel'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Bridge 2D LiDAR from Gazebo to ROS (LaserScan)
    # The lidar publishes directly to /scan in Gazebo (as configured in URDF)
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        world_arg,
        drive_config_arg,
        use_cmd_vel_controller_arg,
        headless_arg,
        world_path_function,
        gz_sim_launch,
        gz_gui,
        create_urdf_action,
        spawn_robot,
        ros_gz_bridge,
        robot_description_launch,
        robot_specifics,
        camera_color_tf,
        camera_color_optical_tf,
        camera_depth_tf,
        camera_depth_optical_tf,
        os_imu_tf,
        os_lidar_tf,
        cmd_vel_controller,
        gps_relay,
        imu_bridge,
        cmd_vel_relay,
        lidar_bridge,
    ])
