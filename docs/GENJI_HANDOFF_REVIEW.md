# Genji(tb3_2) 핸드오프 자료 검토

## 검토 대상

- 원본 zip: `C:\Users\AIN\Downloads\genge-20260709T063234Z-3-001.zip`
- 검토일: 2026-07-09
- 목적: 다른 사람이 실제로 구동해 본 `tb3_2` 관련 자료를 현재 TurtleBot Dashboard 설정과 비교한다.

## 결론

현재 대시보드의 `tb3_2` 기본 설정은 자료의 최신 정본과 대체로 맞다.

- `tb3_2`는 root namespace 기반으로 보는 것이 맞다.
- 주요 주행 topic은 `/cmd_vel`, `/odom`, `/scan`, `/tf`, `/tf_static`, `/amcl_pose`, `/initialpose`, `/navigate_to_pose`다.
- 현재 기준 ROS domain은 `1`로 보는 것이 맞다.
- IP는 DHCP로 바뀔 수 있지만, 자료 기준 대표값은 `192.168.20.7`이다.
- `tb3_1`과 `tb3_2`는 domain과 namespace 방식이 다르다.

## 최신 정본으로 볼 값

자료 안에 서로 충돌하는 값이 있지만, `README.md`, `GENJI-CONNECT.md`, `skill-urhynix-ros-domain-diagnose.md`가 최신 정본 역할을 한다.

| 항목 | Genji / tb3_2 |
|---|---|
| 계정 | `kim` |
| WiFi IP | `192.168.20.7` |
| 유선 IP | `192.168.10.50` |
| ROS_DOMAIN_ID | `1` |
| namespace | root namespace |
| scan | `/scan` |
| odom | `/odom` |
| cmd_vel | `/cmd_vel` |
| amcl pose | `/amcl_pose` |
| initial pose | `/initialpose` |
| Nav2 action | `/navigate_to_pose` |

`tb3_1`은 별도 로봇이다.

| 항목 | T1 / tb3_1 |
|---|---|
| 계정 | `t1` |
| WiFi IP | `192.168.20.101` |
| 유선 IP | `192.168.10.51` |
| ROS_DOMAIN_ID | `2` |
| namespace | `/tb3_1` |

## 문서 안 충돌

`skill-urhynix-genji-nav2-drive.md` 안에는 `ROS_DOMAIN_ID=210`이 남아 있다.

하지만 핸드오프 패키지의 최상위 `README.md`는 과거 문서의 `210/230` 계열 값은 옛 체제라고 명시한다. 따라서 현재 대시보드에서는 `ROS_DOMAIN_ID=1`을 유지한다.

정리:

- 현재 사용: `ROS_DOMAIN_ID=1`
- 과거/스테일 가능성 높음: `ROS_DOMAIN_ID=210`, `ROS_DOMAIN_ID=230`
- `192.168.0.x` 계열 IP도 과거 값으로 간주

## 현재 대시보드와 비교

현재 `config/dashboard_state.json` 기준:

- `activeRobot`: `tb3_2`
- `robotIp`: `192.168.20.7`
- `serverIp`: `192.168.20.3`
- `rosDomainId`: `1`
- `tb3_2.namespace`: `/`
- `tb3_2.cmdVel`: `/cmd_vel`
- `tb3_2.odom`: `/odom`
- `tb3_2.scan`: `/scan`
- `tb3_2.initialPose`: `/initialpose`
- `tb3_2.goalAction`: `/navigate_to_pose`

따라서 기본 주행/수동운전 topic 설정은 자료와 일치한다.

## 카메라 topic 주의

자료에는 카메라 topic 형태가 여러 개 나온다.

현재 대시보드 기본값:

- raw: `/camera/color/image_raw`
- compressed: `/camera/color/image_raw/compressed`

핸드오프 자료의 fullstack 쪽에서 나온 형태:

- compressed: `/tb3_2/camera/image_raw/compressed`

기존 ROS/RealSense 계열에서 나올 수 있는 형태:

- `/camera/image_raw`
- `/camera/image_raw/compressed`
- `/camera/camera/color/image_raw`
- `/camera/camera/color/image_raw/compressed`

대시보드의 로봇 검색 로직은 위 계열 후보를 일부 탐색한다. 카메라가 안 잡히면 먼저 로봇 검색을 돌려서 실제 publisher가 있는 camera topic을 적용해야 한다.

카메라는 topic 이름만 있으면 부족하고, 반드시 publisher count와 마지막 frame 시간을 같이 봐야 한다.

## 주행 bringup 방향

자료 기준 `tb3_2`는 `tb3_1` 방식과 반대다.

- `tb3_2`: 비-ns 단일 Nav2 stack
- `tb3_1`: `/tb3_1` namespaced stack

따라서 `tb3_2`를 `/tb3_2/cmd_vel`로 몰고 가면 안 되는 경우가 많다. 실제 topic list에 `/cmd_vel`, `/odom`, `/scan`이 보이면 root namespace로 유지해야 한다.

## Nav2 관련 주의점

핸드오프 자료의 실제 주행 검증에서 중요하게 본 점:

- Nav2 action server가 보인다고 lifecycle node가 모두 active인 것은 아니다.
- `bt_navigator`, `planner_server`, `controller_server`, `collision_monitor` 상태를 확인해야 한다.
- `collision_monitor`가 inactive면 goal은 받아도 로봇이 안 움직일 수 있다.
- `/initialpose`는 정밀 quaternion으로 발행해야 AMCL이 조용히 거부하지 않는다.
- 시작 goal이 너무 가까우면 Nav2 tolerance 때문에 가짜 성공처럼 보일 수 있다.
- 맵에 없는 다른 로봇이나 큰 물체가 가까이 있으면 AMCL 수렴이 틀어질 수 있다.

## 네트워크 주의점

자료에서 반복해서 강조한 문제:

- DHCP로 IP가 바뀔 수 있다.
- 같은 WiFi여도 AP isolation 때문에 SSH나 ROS discovery가 막힐 수 있다.
- WiFi가 안 되면 유선 직결 IP `192.168.10.50` 경로를 고려한다.
- `ros2 node list`는 daemon cache 때문에 유령 노드를 보여줄 수 있다.

진단할 때는 다음을 같이 본다.

```bash
ros2 daemon stop
ros2 daemon start
ros2 topic list
ros2 topic hz /scan
ros2 topic info /cmd_vel -v
ros2 topic info /odom -v
ros2 topic info /tf -v
```

## 대시보드에 이미 반영된 부분

- `tb3_2` root namespace 설정
- `ROS_DOMAIN_ID=1` 기본값
- `/cmd_vel` type 자동 감지
- `/cmd_vel`을 `Twist` 또는 `TwistStamped`로 자동 전환
- 텔레옵 방식 수동운전 지속 publish
- 로봇 검색 기능
- 진단 로그 복사 기능
- camera on/off
- camera publisher count 확인

## 추가로 개선하면 좋은 부분

1. 로봇 체크에 Nav2 lifecycle 상태 확인 추가
   - `ros2 lifecycle get /controller_server`
   - `ros2 lifecycle get /planner_server`
   - `ros2 lifecycle get /bt_navigator`
   - `ros2 lifecycle get /collision_monitor`

2. 카메라 후보 topic 자동 적용 강화
   - `/tb3_2/camera/image_raw`
   - `/tb3_2/camera/image_raw/compressed`
   - `/camera/image_raw`
   - `/camera/image_raw/compressed`

3. IP drift 대응 버튼 추가
   - 현재 설정 IP ping
   - SSH 연결 확인
   - ROS graph 확인
   - 감지 IP와 설정 IP 비교

4. 보안 개선
   - SSH password는 장기적으로 `dashboard_state.json`에 저장하지 않는 방향이 좋다.
   - 환경변수 또는 local-only secret 파일로 분리한다.

## 현재 판단

이번 자료를 기준으로 볼 때, 지금 우리가 잡은 `tb3_2` 방향은 맞다.

실제 이동이 안 되는 문제는 namespace보다 다음 쪽 가능성이 더 크다.

- `/cmd_vel` subscriber type mismatch
- Nav2 lifecycle 일부 inactive
- `collision_monitor` 또는 controller 쪽 차단
- OpenCR/USB 포트 문제
- `/odom`은 회전만 반영되고 선속도 쪽이 처리되지 않는 문제
- 카메라 driver 미기동 또는 topic 형태 불일치

따라서 다음 실물 테스트 때는 수동운전에서 `/cmd_vel` publish 후 `/odom.twist.twist.linear.x` 변화가 있는지, 그리고 lifecycle 상태가 active인지 먼저 확인한다.
