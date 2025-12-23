cmake_minimum_required(VERSION 3.0.2)
project(wicom_roboarm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
)

add_service_files(
  FILES
  SetAngle.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs std_srvs message_runtime
)

catkin_install_python(PROGRAMS
  src/wicom_roboarm_unified_node.py
  scripts/pca9685_kill_all.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)