#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

pkill -f "python3 .*server.py" 2>/dev/null || true
pkill -f "python .*server.py" 2>/dev/null || true

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export ROS2CLI_NO_DAEMON=1

echo "Starting TurtleBot dashboard"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "URL: http://0.0.0.0:8080"

exec python3 server.py --host 0.0.0.0 --port 8080
