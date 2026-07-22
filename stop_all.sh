#!/usr/bin/env bash
# TurtleBot Dashboard host-side shutdown helper.
# The dashboard updates only the marked config block and never replaces the rest.
set +e

# DASHBOARD_MANAGED_CONFIG_BEGIN
DASHBOARD_CMD_VEL=/cmd_vel
DASHBOARD_CMD_VEL_NAV=/cmd_vel_nav
DASHBOARD_SCAN=/scan
DASHBOARD_ODOM=/odom
DASHBOARD_CAMERA=/camera/image_raw
DASHBOARD_COMPRESSED_CAMERA=/camera/image_raw/compressed
DASHBOARD_NAVIGATION_MANAGER=/lifecycle_manager_navigation
DASHBOARD_LOCALIZATION_MANAGER=/lifecycle_manager_localization
# DASHBOARD_MANAGED_CONFIG_END

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
[ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash"
export ROS2CLI_NO_DAEMON=1
CMD_VEL="${DASHBOARD_CMD_VEL:-/cmd_vel}"
CMD_VEL_NAV="${DASHBOARD_CMD_VEL_NAV:-/cmd_vel_nav}"

publish_zero_velocity() {
  local topic="$1"
  command -v ros2 >/dev/null 2>&1 || return 0
  timeout -k 1 3 ros2 topic pub --once "$topic" geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  timeout -k 1 3 ros2 topic pub --once "$topic" geometry_msgs/msg/TwistStamped \
    "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}" >/dev/null 2>&1 || true
}

stop_helper() {
  local pattern="$1"
  pgrep -f "$pattern" >/dev/null 2>&1 || return 0
  echo "[STOP] host helper: $pattern"
  pkill -INT -f "$pattern" >/dev/null 2>&1 || true
  sleep 1
  pgrep -f "$pattern" >/dev/null 2>&1 || return 0
  pkill -TERM -f "$pattern" >/dev/null 2>&1 || true
}

echo "== dashboard host stop_all.sh =="
echo "[INFO] saved cmd_vel: $CMD_VEL"
echo "[INFO] saved cmd_vel_nav: $CMD_VEL_NAV"
echo "[INFO] official TurtleBot3 topics: scan=$DASHBOARD_SCAN odom=$DASHBOARD_ODOM camera=$DASHBOARD_CAMERA"
echo "[INFO] Nav2 lifecycle: $DASHBOARD_NAVIGATION_MANAGER, $DASHBOARD_LOCALIZATION_MANAGER"
publish_zero_velocity "$CMD_VEL"
publish_zero_velocity "$CMD_VEL_NAV"
for pattern in \
  'robot_yolo_viewer.py' \
  'view_yolo' \
  'view_geng_camera' \
  'rqt_image_view' \
  'webcam_yolo_preview' \
  'webcam_aihub_yolov5' \
  'launch_robot_test.sh' \
  'run_patrol.py' \
  'run_patrol_continuous.py' \
  'run_patrol.sh' \
  'park_.*\.py' \
  'aruco_align_.*\.py' \
  'rotate_t1_precise.py' \
  'rotate_map_yaw.py' \
  'dock_.*' \
  'go_nav2.sh' \
  'go_nav2_geng_rpp' \
  'nav2_rviz.sh' \
  'start_geng_camera'; do
  stop_helper "$pattern"
done
echo "[OK] dashboard host helper stop completed"
