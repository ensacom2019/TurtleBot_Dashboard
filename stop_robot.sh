#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

echo "Publishing repeated zero velocity to /cmd_vel as TwistStamped and Twist."
for _ in $(seq 1 20); do
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}" >/dev/null 2>&1 || true
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  sleep 0.05
done
echo "Stop burst sent."
