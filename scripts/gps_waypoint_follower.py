#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time

from nav2_msgs.action import FollowWaypoints
from geographic_msgs.msg import GeoPoseStamped

import math
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import Quaternion

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


def euler_from_quaternion(q: Quaternion):
    """
    Convert a quaternion into euler angles
    taken from: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    """
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def latLonYaw2Geopose(latitude: float, longitude: float, yaw: float = 0.0) -> GeoPose:
    """
    Creates a geographic_msgs/msg/GeoPose object from latitude, longitude and yaw
    """
    geopose = GeoPose()
    geopose.position.latitude = latitude
    geopose.position.longitude = longitude
    geopose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
    return geopose

class YamlWaypointParser:
    """
    Parse a set of GPS waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_geopose_waypoints(self):
        """
        Get an array of geographic_msgs/msg/GeoPoseStamped objects from the yaml file
        """
        geopose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude = wp["latitude"]
            longitude = wp["longitude"]
            yaw = wp.get("yaw", 0.0)  # Default yaw to 0.0 if not specified
            geopose = latLonYaw2Geopose(latitude, longitude, yaw)
            
            # Create GeoPoseStamped
            geopose_stamped = GeoPoseStamped()
            geopose_stamped.header.frame_id = "wgs84"
            geopose_stamped.pose = geopose
            geopose_wps.append(geopose_stamped)
        
        return geopose_wps


class GpsWaypointFollower(Node):
    """
    Class to use Nav2 GPS waypoint follower to follow a set of waypoints from a yaml file
    Uses the nav2_waypoint_follower action server directly
    """

    def __init__(self, wps_file_path):
        super().__init__('gps_waypoint_follower')
        self.wp_parser = YamlWaypointParser(wps_file_path)
        self.navigator = BasicNavigator("basic_navigator")
        
        # Action client for waypoint follower
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        # Store waypoint count for feedback
        self.total_waypoints = 0

    def wait_for_action_server(self, timeout_sec=10.0):
        """Wait for the waypoint follower action server to be available"""
        self.get_logger().info('Waiting for waypoint follower action server...')
        if not self._action_client.wait_for_server(timeout_sec=rclpy.duration.Duration(seconds=timeout_sec)):
            self.get_logger().error('Waypoint follower action server not available!')
            return False
        self.get_logger().info('Waypoint follower action server available!')
        return True

    def start_waypoint_following(self):
        """
        Function to start the waypoint following
        """
        # Wait for Nav2 to be active
        self.navigator.waitUntilNav2Active(localizer="smoother_server")
        self.get_logger().info('Nav2 is active!')
        
        # Wait for waypoint follower action server
        if not self.wait_for_action_server():
            self.get_logger().error('Failed to connect to waypoint follower action server')
            return
        
        # Get waypoints from YAML file
        geopose_wps = self.wp_parser.get_geopose_waypoints()
        self.get_logger().info(f'Starting to follow {len(geopose_wps)} GPS waypoints')
        
        # Convert GeoPoseStamped to PoseStamped for waypoint follower
        # The waypoint follower expects PoseStamped in map frame
        # We convert GPS coordinates to map coordinates using the robot's current GPS position as origin
        from geometry_msgs.msg import PoseStamped
        from tf2_ros import Buffer, TransformListener
        from sensor_msgs.msg import NavSatFix
        import math
        
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, self)
        
        # Wait a bit for TF to be ready
        time.sleep(1.0)
        
        # Get current robot pose in map frame
        try:
            transform = tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0)
            )
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            self.get_logger().info(f'Current robot position in map: x={current_x:.2f}, y={current_y:.2f}')
        except Exception as e:
            self.get_logger().warn(f'Could not get current robot pose: {e}')
            self.get_logger().warn('Using origin (0,0) as reference for GPS waypoint conversion')
            current_x = 0.0
            current_y = 0.0
        
        # Get current GPS position to use as origin for conversion
        # Subscribe to GPS fix to get current position
        current_gps = None
        gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', lambda msg: setattr(self, '_current_gps', msg), 1
        )
        # Wait for GPS fix
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.5)
            if hasattr(self, '_current_gps') and self._current_gps:
                current_gps = self._current_gps
                break
        
        if current_gps is None:
            self.get_logger().warn('Could not get current GPS position, using first waypoint as origin')
            base_lat = geopose_wps[0].pose.position.latitude
            base_lon = geopose_wps[0].pose.position.longitude
            origin_x = current_x
            origin_y = current_y
        else:
            base_lat = current_gps.latitude
            base_lon = current_gps.longitude
            origin_x = current_x
            origin_y = current_y
            self.get_logger().info(f'Using current GPS position as origin: lat={base_lat:.6f}, lon={base_lon:.6f}')
        
        # Convert GPS waypoints to map frame
        # Convert lat/lon differences to meters relative to current GPS position
        # In ENU frame: X = East (longitude), Y = North (latitude)
        pose_stamped_wps = []
        
        for i, geopose_stamped in enumerate(geopose_wps):
            # Calculate lat/lon differences
            lat_diff = geopose_stamped.pose.position.latitude - base_lat
            lon_diff = geopose_stamped.pose.position.longitude - base_lon
            
            # Convert to meters
            # 1 degree latitude ≈ 111,320 meters (constant)
            lat_meters = lat_diff * 111320.0
            # Longitude conversion depends on latitude (varies with cos(lat))
            # Use average latitude for better accuracy
            avg_lat_rad = math.radians((geopose_stamped.pose.position.latitude + base_lat) / 2.0)
            lon_meters = lon_diff * 111320.0 * math.cos(avg_lat_rad)
            
            # In ENU frame:
            # X axis = East (positive longitude = positive X)
            # Y axis = North (positive latitude = positive Y)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = origin_x + lon_meters  # East = +X
            pose_stamped.pose.position.y = origin_y + lat_meters  # North = +Y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation = geopose_stamped.pose.orientation
            
            pose_stamped_wps.append(pose_stamped)
            self.get_logger().info(
                f'Waypoint {i+1}: GPS ({geopose_stamped.pose.position.latitude:.6f}, '
                f'{geopose_stamped.pose.position.longitude:.6f}) -> Map ({pose_stamped.pose.position.x:.2f}, '
                f'{pose_stamped.pose.position.y:.2f})'
            )
        
        # Store total waypoints for feedback
        self.total_waypoints = len(pose_stamped_wps)
        
        # Send goal to waypoint follower with feedback callback
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = pose_stamped_wps
        
        self.get_logger().info('Sending waypoint following goal...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Waypoint following goal was rejected!')
            return
        
        self.get_logger().info('Waypoint following goal accepted!')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        
        # Monitor until result is ready
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if result_future.done():
                break
        
        # Get result
        result = result_future.result().result
        if result.missed_waypoints:
            self.get_logger().warn(f'Missed {len(result.missed_waypoints)} waypoints')
        else:
            self.get_logger().info('All waypoints completed successfully!')
    
    def feedback_callback(self, feedback_msg):
        """Callback for receiving feedback from waypoint follower"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Executing waypoint {feedback.current_waypoint + 1}/{self.total_waypoints}'
        )


def main():
    rclpy.init()

    # Get default waypoints file path
    default_yaml_file_path = os.path.join(
        get_package_share_directory("avular_kalman_slam"),
        "config",
        "pucp_waypoints.yaml"
    )
    
    # Allow to pass waypoints file as an argument
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    if not os.path.exists(yaml_file_path):
        print(f"Error: Waypoints file not found: {yaml_file_path}")
        print(f"Usage: {sys.argv[0]} [waypoints.yaml]")
        sys.exit(1)

    print(f"Loading waypoints from: {yaml_file_path}")
    gps_wpf = GpsWaypointFollower(yaml_file_path)
    gps_wpf.start_waypoint_following()
    
    # Keep spinning to receive feedback and results
    rclpy.spin(gps_wpf)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

