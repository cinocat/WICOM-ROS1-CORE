#!/usr/bin/env bash
set -euo pipefail

# Detect OS
. /etc/os-release
ID_LIKE_LOWER="$(echo "${ID_LIKE:-$ID}" | tr '[:upper:]' '[:lower:]')"
CODENAME="${VERSION_CODENAME:-}"

echo "Detected OS: ID=${ID}, ID_LIKE=${ID_LIKE:-}, CODENAME=${CODENAME}"

# If a ROS Ubuntu repo accidentally exists on Debian (Clover OS), remove it to avoid EXPKEYSIG and wrong distro
if [[ "${ID}" != "ubuntu" ]] && [[ -f /etc/apt/sources.list.d/ros-latest.list ]]; then
  echo "Removing Ubuntu ROS apt source on non-Ubuntu system..."
  sudo rm -f /etc/apt/sources.list.d/ros-latest.list
fi

echo "Updating package lists..."
sudo apt-get update

COMMON_PKGS=(
  build-essential
  cmake
  pkg-config
  libgstreamer1.0-dev
  libgstreamer-plugins-base1.0-dev
  gstreamer1.0-plugins-base
  gstreamer1.0-plugins-good
  gstreamer1.0-plugins-bad
  gstreamer1.0-plugins-ugly
  gstreamer1.0-libav
  gir1.2-gstreamer-1.0
  gir1.2-gst-plugins-base-1.0
  libgstreamer-plugins-bad1.0-dev
  libgstreamer-plugins-base1.0-dev
  libopencv-dev
  libjsoncpp-dev
  nlohmann-json3-dev
  libwebsocketpp-dev
  libboost-system-dev
  libboost-thread-dev
  nodejs
  npm
)

# On Ubuntu 20.04 (focal / Noetic), libgstreamer-allocators-1.0-dev is available.
if [[ "${ID}" == "ubuntu" && "${CODENAME}" == "focal" ]]; then
  echo "Installing system deps for Ubuntu 20.04 (Noetic)..."
  sudo apt-get install -y "${COMMON_PKGS[@]}" libgstreamer-allocators-1.0-dev
else
  echo "Installing system deps for Debian-based (Clover OS / Raspberry Pi OS)..."
  # Do NOT install libgstreamer-allocators-1.0-dev (not available on Debian)
  sudo apt-get install -y "${COMMON_PKGS[@]}"
fi

# Optional Python tools
sudo apt-get install -y python3-opencv python3-pip
pip3 install --user websockets

echo
echo "Dependency installation complete."
echo "Check if GStreamer WebRTC plugin (webrtcbin) is available:"
echo "  gst-inspect-1.0 webrtc"
echo "If missing, you may need to build gst-plugins-bad with WebRTC enabled."
