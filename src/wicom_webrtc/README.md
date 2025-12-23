```markdown
# webrtc_gst_cpp (ROS C++ WebRTC publisher)

This package provides a ROS C++ node that subscribes to `/main_camera/image_raw` (sensor_msgs/Image),
converts frames to BGR and pushes them into a GStreamer pipeline using appsrc -> vp8enc -> webrtcbin.
Signaling (SDP/ICE) is performed via a WebSocket signaling server (ws://localhost:8080 by default).

Requirements (Ubuntu 20.04, ROS Noetic):
- ROS Noetic (ros-base)
- GStreamer 1.0 and plugins (including webrtc support)
- libgstreamer1.0-dev, libgstreamer-plugins-bad1.0-dev (webrtcbin)
- websocketpp (libwebsocketpp-dev) and Boost (system, thread)
- nlohmann-json (nlohmann-json3-dev)
- OpenCV and cv_bridge

Quick install script:
  webrtc_gst_cpp/scripts/install_deps.sh

Build:
  cd ~/catkin_ws
  catkin_make
  source devel/setup.bash

Run:
  1. Start your Gazebo/Clover simulation (so /main_camera/image_raw publishes).
  2. Start a simple websocket signaling relay (e.g., node signaling/server.js).
  3. Launch:
     roslaunch webrtc_gst_cpp stream.launch signaling_url:=ws://localhost:8080 width:=320 height:=240 fps:=15

Notes test:
- This package uses GStreamer webrtcbin. If your distro's gstreamer-plugins-bad does not include webrtcbin,
  you may need to build gstreamer-plugins-bad with webrtc enabled or use an appropriate PPA.
- The signaling client uses websocketpp. Ensure libwebsocketpp-dev and Boost are available.
- The node is optimized for local simulation (lower fps/resolution possible).
```# webrtc_clvoer_drone
