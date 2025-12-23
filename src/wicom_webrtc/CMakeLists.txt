cmake_minimum_required(VERSION 3.0.2)
project(wicom_webrtc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Silence GStreamer WebRTC unstable API warning
add_definitions(-DGST_USE_UNSTABLE_API)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-webrtc-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
  gstreamer-sdp-1.0
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${GST_LIBRARY_DIRS})
add_definitions(${GST_CFLAGS_OTHER})

add_executable(wicom_webrtc_streamer_node
  src/wicom_webrtc_streamer_node.cpp
  src/wicom_webrtc_adapter.cpp
  src/signaling_client.cpp
)

target_link_libraries(wicom_webrtc_streamer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
  ${Boost_LIBRARIES}
  pthread
)