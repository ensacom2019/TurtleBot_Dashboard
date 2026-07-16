# Robot SSH Bringup

## Multiple robot profiles

Register each robot manually in the dashboard. A profile owns its Robot IP, SSH Host/User/Password, ROS_DOMAIN_ID, and topics. Use a distinct ROS Domain for every robot. Bringup and stop always affect only the currently selected profile; operations on a different robot are independent.

## 목적

대시보드의 `로봇 브링업` 버튼은 서버 PC에서 로봇으로 SSH 접속한 뒤, 로봇 쪽에서 필요한 ROS 노드를 실행하도록 돕는다.

이 기능은 다음을 자동으로 시도한다.

1. 로봇 SSH 접속
2. ROS2 Jazzy 환경 설정
3. 대시보드에서 설정한 맵을 Nav2용 `dashboard_map.pgm` + `dashboard_map.yaml`로 변환
4. 변환한 맵을 로봇의 `~/maps/dashboard/`에 업로드
5. 기본 TurtleBot bringup 확인 및 실행
6. Nav2/AMCL 확인 및 실행 시도
7. 카메라 driver 확인 및 실행 시도
8. ROS graph 출력
9. 대시보드 로봇 체크 재실행

## 전제 조건

셋업 탭에 다음 값이 맞아야 한다.

- Robot SSH Host: `192.168.20.7`
- Robot SSH User: `kim`
- Robot SSH Password: 로봇 계정 비밀번호
- ROS_DOMAIN_ID: `1`
- ROS_LOCALHOST_ONLY: `0`

서버 PC에서 로봇으로 SSH가 가능해야 한다.

```bash
ssh kim@192.168.20.7
```

비밀번호 자동 입력은 다음 순서로 처리한다.

1. 서버 PC에 `sshpass`가 있으면 `sshpass -e` 사용
2. 없고 서버가 Linux/Ubuntu면 pseudo-terminal 방식으로 password prompt에 입력
3. 둘 다 불가능하면 SSH key 인증이 필요

## 로봇에서 실행하는 것

### 대시보드 맵 업로드

브링업 버튼을 누르면 현재 셋업 탭의 맵 설정을 기준으로 Nav2용 맵을 생성한다.

반영되는 항목:

- 현재 `setup.map.imageUrl`
- `resolution`
- `originX`
- `originY`
- `originYaw`
- 검은 벽 이미지
- 수동으로 칠한 `blockedCells`
- 지우개로 뚫은 `freeCells`
- 등록된 박스형 장애물
- 오브젝트 inflation 값

로봇에 저장되는 위치:

```text
~/maps/dashboard/dashboard_map.pgm
~/maps/dashboard/dashboard_map.yaml
```

대시보드 맵을 정상 생성하고 업로드한 경우, Nav2는 기존 로봇 내부 map yaml보다 이 파일을 우선 사용한다.

이미 Nav2가 떠 있어도 대시보드 맵을 업로드한 경우에는 Nav2 관련 노드만 재시작해서 새 맵을 적용한다.

### 기본 bringup

이미 다음 노드가 있으면 재실행하지 않는다.

```text
/turtlebot3_node
/diff_drive_controller
/lidar_node
```

없으면 다음을 detached session으로 실행한다.

```bash
ros2 launch turtlebot3_bringup robot.launch.py usb_port:=<검증된 OpenCR 포트>
```

### Nav2/AMCL

`/navigate_to_pose`와 `/navigate_through_poses` action server가 이미 있으면 재실행하지 않는다.

대시보드 맵 생성이 실패한 경우에는 로봇 안에서 다음 맵 파일을 순서대로 찾는다.

```text
~/maps/arena_shared/arena_shared.yaml
~/maps/map.yaml
~/map.yaml
```

대시보드 맵 또는 기존 맵 파일을 찾으면 다음을 실행한다.

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False map:=<찾은 맵>
```

맵 파일이 없으면 Nav2는 건너뛴다. 이 경우 목표 이동은 계속 불가능하다.

### 카메라

이미 camera publisher가 있으면 재실행하지 않는다.

다음 topic 중 하나에 publisher가 있으면 카메라가 켜진 것으로 본다.

```text
/camera/color/image_raw
/camera/color/image_raw/compressed
/camera/image_raw
/camera/image_raw/compressed
/tb3_2/camera/image_raw/compressed
```

publisher가 없으면 설치된 패키지에 따라 시도한다.

- `turtlebot3_bringup`와 `camera_ros`가 있으면 공식 `camera.launch.py format:=RGB888`
- 위 패키지가 없고 `realsense2_camera`가 있으면 `rs_launch.py`

패키지가 없으면 카메라 bringup은 건너뛴다.

## 실행 결과 확인

브링업 출력은 대시보드의 진단 텍스트 박스에 표시된다.

브링업 후 대시보드가 자동으로 로봇 체크를 다시 실행한다.

## SSH 점검

`SSH 점검` 버튼은 로봇에서 무언가를 새로 켜지 않고, 로봇 내부 상태를 직접 읽어온다.

확인하는 항목:

- 로봇 IP와 네트워크 인터페이스
- ROS 환경값
- 실행 중인 TurtleBot/Nav2/카메라 관련 프로세스
- tmux 세션
- `/dev/ttyACM*`, `/dev/ttyUSB*` 장치와 vendor 정보
- `~/maps/dashboard/dashboard_map.yaml`
- `ros2 node list`
- `ros2 topic list`
- `ros2 action list`
- 주요 topic의 `ros2 topic info -v`
- `/scan`, `/odom`, camera topic의 `ros2 topic hz`
- Nav2 lifecycle node 상태
- `~/turtlebot_dashboard_logs` 로그와 실행 스크립트 tail

이 결과는 `진단 복사`에도 함께 포함된다. 서버 PC에서는 안 보이는데 로봇 안에서는 보이는 경우, 또는 반대로 로봇 안에서는 정상인데 서버 PC에서 안 보이는 경우를 구분하는 데 사용한다.

목표 이동이 가능하려면 다음이 되어야 한다.

```bash
ros2 action info /navigate_to_pose
ros2 action info /navigate_through_poses
# Action servers: 1

ros2 topic info /amcl_pose -v
# Publisher count: 1

ros2 topic info /initialpose -v
# Subscription count: 1 이상
```

카메라가 가능하려면 다음 중 하나가 되어야 한다.

```bash
ros2 topic info /camera/color/image_raw -v
ros2 topic info /camera/color/image_raw/compressed -v
# Publisher count: 1 이상
```

## 한계

- 맵 파일이 로봇에 없으면 Nav2는 자동으로 켜지지 않는다.
- camera driver 패키지가 없으면 카메라는 자동으로 켜지지 않는다.
- `/dev/ttyACM1`을 우선 검사하고, 없거나 OpenCR이 아니면 `/dev/ttyACM0`을 검사한다.
- `udevadm`의 OpenCR 모델/시리얼 정보가 확인되고 Arduino Uno가 아닐 때만 로봇 드라이버를 실행한다.
- 실행 후 `/turtlebot3_node`가 ROS graph에 나타나지 않으면 브링업 실패로 판정하고 `tb3_base.log`를 출력한다.
- 이미 다른 사용자가 띄운 충돌 노드가 있으면 먼저 정리해야 할 수 있다.
- password를 설정 파일에 저장하는 방식은 장기적으로 환경변수나 local secret 파일로 분리하는 것이 좋다.

## 권장 사용 순서

1. 서버 PC에서 대시보드 실행
2. 셋업 탭에서 SSH/IP/domain 확인
3. `로봇 브링업` 클릭
4. 출력에서 `dashboard map uploaded` 확인
5. `로봇 체크` 결과 확인
6. 수동운전 테스트
7. Nav2/AMCL이 정상일 때 목표 이동 테스트
