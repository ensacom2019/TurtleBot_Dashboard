# TurtleBot Dashboard

TurtleBot3 Burger와 ROS 2 Jazzy를 위한 로컬 웹 대시보드입니다. 지도 설정, A* 경로 계획, LiDAR 기반 비상 주행, 수동 운전, 카메라 확인, 로봇 SSH 브링업을 한 화면에서 다룹니다.

## 다중 로봇 (서로 다른 ROS Domain)

자동 네트워크 검색은 사용하지 않습니다. 셋업의 `+` 버튼으로 로봇 프로필을 직접 추가하고, **각 프로필별로** Robot IP, SSH Host/User/Password, `ROS_DOMAIN_ID`를 입력해 저장합니다. 각 로봇은 반드시 서로 다른 Domain을 사용하며, 토픽 이름은 모두 `/scan`, `/amcl_pose`처럼 같아도 됩니다.

프로필을 선택하면 그 로봇만 제어 대상이 됩니다. `선택 로봇 브링업`과 `선택 로봇 종료`는 선택한 프로필의 SSH와 Domain으로만 동작합니다. 다른 프로필은 Domain별 ROS worker가 pose를 구독하여 지도상의 회피 장애물로 반영하며, pose를 아직 받지 못한 경우에는 수동 위치값을 사용합니다.

## 화면

### 셋업 대시보드

지도, 벽, 장애물, 로봇 footprint와 안전 반경을 한 화면에서 설정합니다.

![TurtleBot Dashboard setup screen](docs/images/dashboard-overview.png)

### 주행 대시보드

카메라 영상과 FPS, 지도 위 로봇 위치, 계획 경로와 LiDAR 점을 함께 확인합니다. 우측 패널에서 수동 운전, 목표 및 경유지 지정, 반복 운행, A* 직접 주행을 제어하고 주행 로그를 수집할 수 있습니다.

![TurtleBot Dashboard drive screen](docs/images/dashboard-drive.png)

### 맵 제작 대시보드

맵의 실제 크기와 `1픽셀당 cm` 비율로 그리드를 생성합니다. 펜과 지우개로 벽을 편집하고, 마우스 휠 확대·축소와 `Shift + 드래그` 이동을 사용해 큰 맵도 다룰 수 있습니다. 완성된 맵은 내부 맵 폴더에 저장한 뒤 셋업 화면에서 선택합니다.

![TurtleBot Dashboard map editor](docs/images/dashboard-map-editor.png)

## 주요 기능

- 셋업 / 주행 / 맵 제작 탭
- 저장된 맵 선택과 기본 맵 전환
- 맵 제작: cm 단위 크기, 픽셀 해상도, 벽 그리기/지우기, 확대/이동, 저장/불러오기/삭제
- 맵 제작본은 로컬 `data/maps/`에 저장되고 Git에서는 제외
- 초기 위치, 로봇 크기, 부속품 돌출, 장애물 크기와 위치 설정
- 목표점과 경유지 기반 A* 경로 생성, 최종 방향 지정, 반복 운행
- Nav2 사용 가능 시 Nav2 실행, 불가능 시 LiDAR A* 직접 추종 사용
- LiDAR 점 표시와 본체 외곽 거리 기반 감속/회피
- 카메라 화면, 수동 주행, 주행 로그, 진단 보고서
- ROS 2 토픽 설정, 로봇 탭 선택, 같은 사설 네트워크의 SSH 장비 및 ROS 로봇 검색, 로봇 체크
- SSH 브링업: OpenCR 포트 검증 후 systemd 사용자 서비스로 TurtleBot base를 실행하고 Nav2/AMCL, 카메라를 시작
- SSH 브링업 종료: Nav2 lifecycle shutdown과 Ctrl+C(SIGINT) 후 검증을 거쳐 base·LiDAR·Nav2·카메라를 정상 종료

## 요구 사항

- Python 3.10 이상
- ROS 2 Jazzy: 실제 로봇 연결 및 ROS 브리지 사용 시 필요
- TurtleBot3 Burger와 OpenCR
- 같은 네트워크와 동일한 `ROS_DOMAIN_ID`

Windows에서는 ROS 없이도 프리뷰 모드로 UI와 맵 제작 기능을 사용할 수 있습니다.

## 실행

### Ubuntu / ROS 2 Jazzy

```bash
chmod +x run_ubuntu.sh stop_dashboard.sh stop_robot.sh check_camera.sh
./run_ubuntu.sh
```

또는 직접 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
python3 server.py --host 0.0.0.0 --port 8080
```

### Windows 프리뷰

```powershell
python server.py --host 127.0.0.1 --port 8080
```

브라우저에서 `http://127.0.0.1:8080/`을 엽니다.

## 맵 제작

1. `맵 제작` 탭에서 맵 이름, 가로/세로 cm, `1픽셀당 cm`을 설정합니다.
2. `그리드 생성`을 누릅니다.
3. 펜으로 벽을 그리고 지우개로 벽을 지웁니다.
4. 휠로 확대/축소하고 `Shift + 드래그`로 화면을 이동할 수 있습니다.
5. `맵 저장`을 누르면 `data/maps/`에 PNG로 저장되고 저장된 맵 목록에 추가됩니다.
6. 저장된 맵을 선택한 뒤 `불러오기`를 누르면 다시 편집할 수 있습니다.

기본값은 `180 x 180cm`, `180 x 180px`, `1px = 1cm`입니다. 예를 들어 `1픽셀당 cm`을 `10`으로 설정하면 180cm 맵은 18px로 생성되며, 저장 맵의 해상도는 `0.1m/px`가 됩니다.

## ROS 2 기본 토픽

기본 로봇 프로필은 루트 namespace의 다음 토픽을 사용합니다.

| 기능 | 기본 토픽 |
| --- | --- |
| LiDAR | `/scan` |
| Pose | `/amcl_pose` |
| Odometry | `/odom` |
| 수동 주행 | `/cmd_vel` |
| 초기 위치 | `/initialpose` |
| 단일 목표 | `/navigate_to_pose` |
| 경유지 목표 | `/navigate_through_poses` |
| 카메라 | `/camera/image_raw` |

`tb3_1` 프로필은 `/tb3_1/...` namespace를 사용합니다. 토픽명과 네트워크 값은 셋업 탭에서 변경할 수 있습니다.

## 운행 방식

1. 지도와 본체 footprint, 장애물을 기반으로 브라우저에서 A* 경로를 계산합니다.
2. Nav2 action server가 정상일 때는 Nav2로 목표/경유지를 전달합니다.
3. Nav2가 없거나 테스트 토글을 켠 경우에는 서버가 A* 경로를 Pure Pursuit 방식으로 직접 추종합니다.
4. LiDAR와 odometry가 최신 상태인지 확인하고 `/cmd_vel` publisher 충돌을 방지합니다.
5. LiDAR 점이 본체 외곽 3cm 이내로 들어오면 빈 방향을 찾아 회피합니다. 감속 구간에서는 설정된 감속 속도로 일정하게 주행합니다.

실제 주행 전에는 낮은 속도와 넓은 안전 여유로 테스트하고, `/scan`, `/odom`, TF, `/cmd_vel` 수신 상태를 로봇 체크에서 확인하세요.

## 로봇 SSH 브링업

셋업 탭에 로봇 IP, SSH 계정, ROS domain을 입력한 뒤 `로봇 브링업`을 누르면 다음을 시도합니다.

- `/dev/ttyACM1`, `/dev/ttyACM0`에서 OpenCR 식별 및 Arduino Uno 제외
- `turtlebot-dashboard-base.service` 사용자 서비스로 TurtleBot3 base bringup
- 현재 선택한 대시보드 맵을 ROS map 파일로 전송
- Nav2/AMCL과 카메라 bringup

`로봇 브링업`은 SSH 사용자의 systemd 서비스를 로그인 종료 후에도 유지하도록 linger 설정을 확인합니다. 꺼져 있으면 먼저 비밀번호 없는 sudo를 시도하고, 이어서 셋업에 저장된 SSH 비밀번호로 자동 활성화합니다. 로봇 계정에 sudo 권한이 없거나 sudo 비밀번호가 SSH 비밀번호와 다를 때만 로봇에서 아래 명령을 한 번 직접 실행하면 됩니다.

```bash
sudo loginctl enable-linger kim
```

`브링업 종료`는 먼저 정지 명령을 보내고 Nav2 lifecycle manager에 shutdown을 요청합니다. 이어 dashboard가 만든 Nav2·카메라 tmux 세션에 `Ctrl+C`를 보내 최대 12초간 정상 종료를 기다립니다. base는 `systemctl --user stop turtlebot-dashboard-base.service`로 service cgroup 전체에 `SIGINT`를 전달하므로 LiDAR 자식 프로세스도 `Ctrl+C`와 같은 순서로 종료됩니다. 시간 안에 끝나지 않은 dashboard tmux/nohup 세션만 제한적으로 강제 종료하며, 수동으로 실행한 ROS 프로세스는 종료하지 않고 검증 실패로 표시합니다.

직접 SSH 터미널에서 실행한 기존 `ros2 launch`는 대시보드가 소유하지 않으므로 자동 종료하지 않습니다. 한 번 `Ctrl+C`로 종료한 뒤 대시보드 브링업으로 전환하세요.

## 검증

```bash
python3 -m py_compile server.py
python3 -m unittest discover -s tests -v
node --check web/app.js
```

## 저장 파일과 보안

- 실행 중 설정은 `config/dashboard_state.json`에 저장됩니다.
- 사용자 제작 맵은 `data/maps/`에 저장됩니다.
- 주행 로그는 `run_logs/`에 저장됩니다.
- 위 파일과 SSH 비밀번호가 포함될 수 있는 로컬 설정은 `.gitignore`로 Git에서 제외됩니다.

프로젝트 구조와 상세 자료는 `docs/`를 참고하세요.
