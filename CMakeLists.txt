cmake_minimum_required(VERSION 3.5)
project(avular_kalman_slam)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY  launch rviz config world
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/gps_waypoint_follower.py scripts/gps_waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

