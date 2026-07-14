#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

echo "== ROS environment =="
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"

echo
echo "== Camera-like nodes =="
ros2 node list | grep -Ei "camera|realsense|usb_cam|v4l2|picamera|raspicam|image_proc" || true

echo
echo "== Camera topics =="
ros2 topic list | grep -Ei "camera|image|compressed" || true

echo
echo "== Topic info: raw =="
ros2 topic info /camera/color/image_raw -v || true

echo
echo "== Topic info: compressed =="
ros2 topic info /camera/color/image_raw/compressed -v || true

echo
echo "== Topic rate: raw =="
timeout 8 ros2 topic hz /camera/color/image_raw --window 5 || true

echo
echo "== Topic rate: compressed =="
timeout 8 ros2 topic hz /camera/color/image_raw/compressed --window 5 || true
