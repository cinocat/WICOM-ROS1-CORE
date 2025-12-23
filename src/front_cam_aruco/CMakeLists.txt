cmake_minimum_required(VERSION 3.0.2)
project(front_cam_aruco)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2_ros
  geometry_msgs
  sensor_msgs
  std_msgs
  cv_bridge
  image_proc
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/front_aruco_detect.py
  scripts/calculate_front_angle.py
  scripts/aruco_front_overlay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config www
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)