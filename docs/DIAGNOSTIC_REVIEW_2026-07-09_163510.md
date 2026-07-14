# Diagnostics Review - 2026-07-09 16:35:10

## 결론

이 진단 기준으로 로봇과 서버의 기본 ROS 통신은 되고 있다.

정상인 것:

- `ROS_DOMAIN_ID=1`
- 서버 IP 감지 `192.168.20.4`
- `/scan` publisher 있음
- `/odom` publisher 있음
- `/cmd_vel` subscriber 있음
- `/cmd_vel` subscriber type은 `geometry_msgs/msg/TwistStamped`
- `/tf`, `/tf_static` 있음
- root namespace 구조가 맞음

문제인 것:

- Nav2 action server가 없다.
- Nav2 lifecycle node가 없다.
- AMCL publisher가 없다.
- `/initialpose` subscriber가 없다.
- 카메라 publisher가 없다.
- camera node가 없다.
- `/scan` publisher는 있는데 대시보드의 `lastScanAt`이 비어 있었다.

## 실제 ROS graph 상태

보이는 노드:

```text
/diff_drive_controller
/lidar_node
/robot_state_publisher
/turtlebot3_node
/turtlebot_web_dashboard
```

즉, 기본 bringup 계열만 떠 있다.

보이지 않는 Nav2/AMCL 노드:

```text
/map_server
/amcl
/controller_server
/planner_server
/bt_navigator
/behavior_server
/collision_monitor
/lifecycle_manager_navigation
```

따라서 목표 이동은 동작할 수 없다.

## 왜 `/navigate_to_pose`가 보이는데 실패인가

진단에는 다음이 같이 나온다.

```text
ros2 action list:
/navigate_to_pose

ros2 action info /navigate_to_pose:
Action clients: 1
  /turtlebot_web_dashboard
Action servers: 0
```

`/navigate_to_pose` 이름이 보이는 이유는 대시보드가 action client를 만들었기 때문이다.

중요한 것은 action server 수다. `Action servers: 0`이면 Nav2가 goal을 받을 서버가 없는 상태다.

## 왜 초기 위치가 실제 AMCL에 적용되지 않는가

진단에는 다음이 나온다.

```text
/amcl_pose publishers 0
/initialpose subscribers 0
```

즉 AMCL이 실행 중이 아니다.

이 상태에서 초기 위치를 지정하면 로봇 localization에는 적용되지 않고, 대시보드가 `/odom` 기준 표시 좌표를 보정하는 데만 쓰인다.

## 왜 카메라가 안 나오는가

진단에는 camera topic 이름은 보인다.

```text
/camera/color/image_raw
/camera/color/image_raw/compressed
```

하지만 publisher가 없다.

```text
raw publishers 0
compressed publishers 0
camera nodes -
```

이 topic들은 대시보드가 subscriber를 만들었기 때문에 topic list에 보일 수 있다. 실제 카메라 드라이버가 publish 중이라는 뜻은 아니다.

카메라를 보려면 camera driver node를 따로 bringup해야 한다.

## `/scan` 주의점

진단에서 `/scan` publisher는 있다.

```text
Publisher: lidar_node
QoS Reliability: BEST_EFFORT
```

기존 대시보드는 기본 reliable subscription을 사용해서 `lastScanAt`이 비어 있을 수 있었다.

`v2026-07-09.11`부터 `/scan`, raw image, compressed image subscriber는 ROS sensor data QoS를 사용하도록 수정했다.

## `/cmd_vel` 상태

로봇 쪽 `/cmd_vel`은 다음 타입을 구독한다.

```text
geometry_msgs/msg/TwistStamped
```

대시보드는 이 타입을 자동 감지해서 `TwistStamped`로 발행해야 한다.

진단의 Robot Check에는 다음이 나온다.

```text
cmd_vel message type: dashboard geometry_msgs/msg/TwistStamped
subscriber types geometry_msgs/msg/TwistStamped
```

따라서 topic/type 방향은 맞다.

수동운전이 안 된다면 다음을 확인한다.

```bash
ros2 topic echo /odom --once
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped ...
ros2 topic hz /odom
```

대시보드가 `/cmd_vel`을 보낸 뒤 `/odom.twist.twist.linear.x`가 변하는지 봐야 한다.

## 다음 조치

### 수동운전

1. 대시보드를 `v2026-07-09.11`로 재시작한다.
2. 로봇 체크에서 `/cmd_vel` type이 `TwistStamped`인지 확인한다.
3. 수동운전 전진을 누른다.
4. `/odom`의 위치 또는 twist가 변하는지 확인한다.

### 목표 이동

현재 상태에서는 목표 이동이 안 되는 것이 정상이다. Nav2/AMCL을 올려야 한다.

예상 필요 상태:

```text
/map_server
/amcl
/controller_server
/planner_server
/bt_navigator
/behavior_server
/collision_monitor
/lifecycle_manager_navigation
```

그리고 다음이 되어야 한다.

```bash
ros2 action info /navigate_to_pose
# Action servers: 1

ros2 topic info /amcl_pose -v
# Publisher count: 1

ros2 topic info /initialpose -v
# Subscription count: 1 이상
```

### 카메라

카메라 driver를 별도로 bringup한다.

확인 기준:

```bash
ros2 topic info /camera/color/image_raw -v
ros2 topic info /camera/color/image_raw/compressed -v
ros2 topic hz /camera/color/image_raw --window 5
```

publisher가 없으면 대시보드 문제가 아니라 카메라 bringup이 안 된 상태다.
