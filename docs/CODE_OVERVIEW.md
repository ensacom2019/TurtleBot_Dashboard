# TurtleBot3 Dashboard 코드 설명

## 전체 구조

이 프로젝트는 Python 단일 서버와 브라우저 프론트엔드로 구성된다.

```text
TurtleBot/
  server.py                  Python HTTP server + ROS2 bridge
  web/
    index.html               화면 구조
    app.js                   UI 상태, 지도 편집, A* 경로계획, API 호출
    styles.css               화면 스타일
  config/
    dashboard_state.json     셋업값과 런타임 상태 저장
  data/
    Sprite-1.png             현재 맵 이미지
  docs/
    IMPROVEMENT_ROADMAP.md   개선 방향
    CODE_OVERVIEW.md         코드 설명
```

실행하면 `server.py`가 HTTP 서버를 열고, 브라우저는 `web/index.html`, `web/app.js`, `web/styles.css`를 받아 화면을 구성한다. ROS2가 있는 환경에서는 `rclpy`로 실제 ROS graph에 연결하고, ROS2가 없는 환경에서는 preview mode로 UI만 확인한다.

## 실행 흐름

1. `server.py` 실행
2. `DashboardState`가 `config/dashboard_state.json`을 읽어 현재 셋업을 로드
3. `RosBridge`가 ROS2 모듈 import 시도
4. ROS2 사용 가능 시 publisher/subscriber/action client 생성
5. 브라우저가 `/api/state` 또는 `/api/events`로 상태를 받아 화면 표시
6. 사용자가 셋업/주행 조작
7. 브라우저가 `/api/...` 엔드포인트로 명령 전송
8. 서버가 설정 저장, ROS topic publish, Nav2 action send, camera frame update 등을 처리

## `server.py`

`server.py`는 크게 네 부분으로 나뉜다.

### 1. 기본 설정과 유틸 함수

주요 항목:

- `APP_VERSION`: 대시보드 버전 표시용 문자열
- `DEFAULT_STATE`: 초기 셋업과 런타임 기본값
- `topics_for_namespace()`: namespace 기준 topic 묶음 생성
- `namespace_topic()`, `namespace_from_topic()`: topic prefix 처리
- `detect_server_ip()`, `select_server_ip()`: 로봇과 같은 대역 또는 라우팅 인터페이스 기준 서버 IPv4 자동 선택
- `diagnostic_commands_for_topics()`: 진단용 ROS 명령 목록 생성
- `format_diagnostics_report()`: 복사 가능한 진단 리포트 생성
- `RunLogStore`: 세션별 JSONL, 최근 4,000개 이벤트, 5 MiB 제한과 Markdown 내보내기 관리
- `sanitize_run_log_value()`: 로그의 비밀번호/secret 마스킹과 크기·깊이 제한

이 영역은 UI 기본값, topic 이름, 진단 로그 형식을 바꿀 때 수정한다.

### 2. `DashboardState`

역할:

- `config/dashboard_state.json` 로드
- 셋업값 저장
- 런타임 상태 저장
- 카메라 frame bytes 저장
- camera on/off 상태 관리

중요 메서드:

- `snapshot()`: 현재 전체 상태 복사본 반환
- `get_setup()`: setup 영역만 반환
- `update_setup(patch)`: 셋업 변경 후 파일 저장
- `update_runtime(patch)`: 런타임 상태만 갱신
- `set_camera_enabled(enabled)`: 카메라 표시 on/off
- `set_camera(content, content_type)`: 최신 카메라 frame 저장
- `get_camera()`: 브라우저에 줄 camera frame 반환

셋업 탭에서 저장한 값은 대부분 이 클래스를 통해 `dashboard_state.json`에 반영된다.

### 3. `NullRosBridge`

ROS2가 없거나 `rclpy` import에 실패했을 때 사용하는 preview bridge다.

역할:

- Windows에서 UI 확인 가능
- 실제 ROS publish 없이 화면 상태만 변경
- 목표 이동과 수동운전을 preview 상태로 표시
- 카메라 placeholder 표시

프리뷰 화면에서 로봇이 실제로 움직이는 것처럼 보일 수 있지만, 이 모드는 실물 로봇에 명령을 보내지 않는다.

### 4. `RosBridge`

실제 ROS2 연결을 담당하는 핵심 클래스다.

초기화 때 하는 일:

- `rclpy` 초기화
- dashboard node 생성
- `/initialpose` publisher 생성
- `/goal_pose` publisher 생성
- `/cmd_vel` publisher 생성
- `/amcl_pose`, `/odom`, `/scan`, camera topic subscriber 생성
- Nav2 `NavigateToPose`, `NavigateThroughPoses` action client 생성
- LaserScan 거리점 보관과 LiDAR 비상주행 제어 상태 초기화
- ROS spin thread 시작
- 수동운전 watchdog thread 시작

중요 메서드:

- `reload_setup()`: topic 설정 변경 시 ROS node 재생성
- `set_initial_pose(x, y, yaw)`: `/initialpose` 발행
- `send_goal(x, y, yaw, path)`: 단일 목표를 Nav2로 보내고 Nav2가 없으면 A* path를 LiDAR 비상주행에 전달
- `send_route(poses, path, repeat)`: `NavigateToPose`를 경유지마다 순차 실행하고, 번호 구간 반복과 휴식 상태를 서버에서 관리
- `_dispatch_route_pose()`: ROS result callback과 분리된 스레드에서 다음 `NavigateToPose` 목표 전송
- `_on_route_pose_result()`: 경유지 성공 시 다음 지점으로 전환하고 마지막 지점이면 완료 또는 반복 휴식으로 전환
- `_schedule_route_repeat()` / `_resume_route_repeat()`: 휴식 중 세대 번호를 확인하고 취소되지 않은 경로만 첫 지점부터 재시작
- `_dispatch_route_action()`: `NavigateToPose`가 없을 때 `NavigateThroughPoses` 실행 및 반복 재전송
- `_start_lidar_fallback()`: 최대 2.5초 동안 최신 scan/odom/pose를 기다리고 cmd_vel 제어권 확인 후 경로 추종 시작
- `_wait_for_fresh_fallback_sensors()`: ROS 재발견이나 Wi-Fi 지연으로 첫 센서 샘플이 늦을 때 짧게 대기하고 자동 재검사
- `_queue_lidar_fallback_until_fresh()`: 첫 대기 이후에도 센서가 stale이면 목표를 최대 15초간 보존
- `_pending_lidar_fallback_loop()`: 대기 중 센서가 복구되면 저장된 A* 경로를 자동 시작하고 취소/수동운전 요청 시 폐기
- `_fallback_navigation_loop()`: Pure Pursuit 명령, 감속, 예상 footprint 충돌 회복을 10Hz로 수행
- `evaluate_lidar_safety()`: LaserScan 점과 미래 직사각형 footprint의 충돌 및 속도 배율 계산
- `sample_lidar_points()`: 충돌 검사용 원본과 별도로 웹 표시점을 최대 240개로 제한하고 좌표 반올림
- `choose_lidar_recovery_turn()`: 좌우 LiDAR 여유를 비교해 회전 방향 선택
- `lidar_recovery_motion_safe()`: 회전·전진·후진 후보를 실제 footprint로 사전 검증하고, 이미 경계가 겹친 경우에는 겹침을 키우지 않으면서 최종 여유가 증가하는 탈출 동작만 허용
- `next_lidar_recovery_command()`: 정지 → 양방향 회전/후진/전진 → 경로 재합류 상태를 관리하고, 안전 후보가 없던 trapped 상태도 최신 scan으로 주기적으로 재검사
- `cancel_goal()`: 현재 Nav2 goal 취소
- `stop_robot()`: 수동운전 watchdog 해제 후 0속도 여러 번 발행
- `manual_drive(linear, angular)`: 수동운전 목표 속도 설정
- `manual_drive_check()`: 0속도 명령을 발행하고 SSH의 `ros2 topic echo`로 로봇 수신 확인
- `connection_status()`: 연결 상태 요약
- `robot_check()`: 토픽, TF, 카메라, Nav2 상태 점검
- `diagnostics_report()`: 복사 가능한 전체 진단 리포트 생성

## 수동운전 코드 흐름

수동운전은 텔레옵 방식으로 동작한다.

브라우저 쪽:

1. 방향 버튼을 클릭하거나 방향 키 입력 발생
2. `startManualDrive(linearFactor, angularFactor)` 호출
3. `/api/manual_drive`로 목표 속도 전송
4. 0.4초마다 유지 신호 전송
5. 정지 버튼, `Space`, 셋업 탭 이동, focus 상실 시 0속도 전송

서버 쪽:

1. `DashboardHandler.do_POST()`가 `/api/manual_drive` 요청 수신
2. `RosBridge.manual_drive(linear, angular)` 호출
3. 진행 중인 Nav2 goal 취소
4. 속도 제한 적용
5. non-zero 명령이면 `_arm_manual_watchdog()`로 목표 속도 저장
6. `_manual_watchdog_loop()`가 약 10Hz로 `/cmd_vel` 지속 발행
7. 유지 신호가 1.2초 이상 끊기면 0속도 여러 번 발행

`/cmd_vel` message type은 `_ensure_cmd_vel_publisher_type()`에서 실제 subscriber type을 보고 자동으로 `Twist` 또는 `TwistStamped`를 선택한다. `TwistStamped` 시간은 최근 `/odom` 헤더와 단조 시계를 결합해 로봇 기준으로 생성한다.

## 카메라 코드 흐름

ROS 쪽:

- raw image topic subscriber가 `_on_image()` 호출
- compressed image topic subscriber가 `_on_compressed_image()` 호출
- frame이 들어오면 `DashboardState.set_camera()`에 저장

브라우저 쪽:

- `startCameraLoop()`가 `/api/camera/frame`을 주기적으로 요청
- 받은 이미지를 카메라 영역에 표시
- `/api/camera_control`로 카메라 표시 on/off 변경

주의할 점:

- topic 이름이 있다고 카메라가 정상인 것은 아니다.
- publisher count와 마지막 frame 시간이 같이 확인되어야 한다.

## API 엔드포인트

GET:

- `/api/state`: SSH 비밀번호를 제외한 dashboard state 반환
- `/api/connection`: ROS 연결 상태 반환
- `/api/robot_check`: 로봇 체크 실행
- `/api/diagnostics`: 진단 리포트 생성
- `/api/run_logs`: 현재 주행 세션 요약과 Markdown 리포트 조회
- `/api/run_logs/download`: 현재 주행 로그 Markdown 파일 다운로드
- `/api/discover`: ROS graph 기반 로봇 후보 검색
- `/api/events`: Server-Sent Events 상태 스트림
- `/api/camera/frame`: 최신 카메라 frame 반환

POST:

- `/api/setup`: 셋업값 저장
- `/api/initial_pose`: 초기 위치 설정 및 `/initialpose` 발행
- `/api/goal`: 단일 목표 이동
- `/api/route`: 경유지 포함 목표 이동
- `/api/run_logs/clear`: 기존 파일을 보존하고 새 주행 로그 세션 시작

`/api/goal`과 `/api/route`의 `forceFallback: true`는 Nav2 Action 검색을 건너뛰고 A* 경로를 `_start_lidar_fallback()`으로 직접 전달한다. 센서와 `/cmd_vel` 안전 조건은 우회하지 않는다.
- `/api/cancel`: 현재 goal 취소
- `/api/stop`: 로봇 정지
- `/api/robot_bringup`: 현재 셋업 맵을 Nav2 map으로 변환해 SSH로 로봇에 업로드하고 기본 bringup/Nav2/카메라 실행 시도 후 로봇 체크
- `/api/robot_ssh_check`: SSH로 로봇 내부 node/topic/action/lifecycle/USB/log 상태 점검
- `/api/manual_drive`: 수동운전 속도 설정
- `/api/manual_drive_check`: 0속도 `cmd_vel`의 로봇 SBC 수신 여부 점검
- `/api/camera_control`: 카메라 표시 on/off

## `web/app.js`

프론트엔드의 대부분 로직이 들어 있다.

### 상태와 DOM

- `state`: 브라우저 내부 상태
- `els`: HTML element 참조 모음
- `bindElements()`: HTML element를 `els`에 연결
- `bindTabs()`: 셋업 탭과 주행 탭 전환
- `bindActions()`: 버튼 이벤트 연결
- `connectEvents()`: 서버 상태 스트림 연결
- `applyState(data)`: 서버 state를 화면에 반영

셋업을 편집하는 동안에는 `applyState()`가 로컬 `setup`을 보존하고 SSE의 `runtime`만 반영한다. 저장 응답이 도착해 `setupDirty`가 해제된 뒤에만 서버 setup으로 교체하므로 오브젝트 이동 좌표와 벽 편집 데이터가 주기적인 SSE에 의해 되돌아가지 않는다.

### 셋업 폼

관련 함수:

- `fillSetupForm(setup)`: setup 값을 input에 채움
- `readSetupForm()`: input 값을 setup 객체로 변환
- `setupInputs()`: 셋업 input event 연결
- `markSetupDirty()`: 저장 필요 상태 표시
- `syncSetupShadow()`: 서버 state와 로컬 setup 동기화

셋업 탭에서 입력한 맵, 로봇 크기, 장애물, topic 설정은 `readSetupForm()`을 거쳐 `/api/setup`으로 저장된다. 저장할 때 초기 위치가 변경된 경우 서버가 `/initialpose` 발행과 odom 표시 기준점 설정까지 함께 수행한다. 초기 위치가 바뀌지 않은 일반 설정 저장은 현재 로봇 위치를 초기값으로 덮어쓰지 않는다.

### 맵 좌표 변환

중요 함수:

- `mapMetrics(setup)`: 맵 grid 크기와 cell 정보를 계산
- `mapResolution()`: meter/pixel 해상도
- `mapOrigin()`: 맵 원점
- `worldToGrid(world, setup)`: meter 좌표를 grid cell로 변환
- `gridToWorld(cell, setup)`: grid cell을 meter 좌표로 변환
- `worldToCanvas(worldX, worldY, canvas)`: meter 좌표를 canvas 좌표로 변환
- `canvasToWorld(canvasX, canvasY, canvas)`: canvas 좌표를 meter 좌표로 변환

맵 클릭, 장애물 이동, 초기 위치 지정, 목표 지정은 모두 이 좌표 변환을 거친다.

### 장애물과 벽

관련 함수:

- `normalizeObstacles()`: 장애물 데이터 보정
- `renderObstacleList()`: 장애물 목록 UI 렌더링
- `addObstacle()`, `moveObstacle()`, `deleteObstacle()`: 장애물 편집
- `mapWallCellSet()`: 검은색 벽 이미지를 grid cell로 변환
- `blockedCellSet()`: 수동 벽 cell과 이미지 벽 cell 병합
- `obstacleCellKeys()`: 박스형 장애물을 grid cell로 변환
- `inflatedCellSet()`: 로봇 외접 footprint를 반영한 절대 진입 금지 cell 계산
- `softInflatedCellSet()`: 진입 금지 영역 바깥의 감속 비용 cell 계산
- `plannerClearanceRadius()`: 사용자가 지정한 0.05~0.40m 분홍 금지 반경과 장애물 추가 여유를 계산
- `bindSynchronizedRange()`: 분홍 금지 반경, Inflation, Scan/Odom 제한 슬라이더를 숫자 입력과 동기화

장애물 크기는 cm 기준 셋업값을 meter 단위로 저장하고, 화면에서는 map transform을 통해 실제 비율로 그린다.

### A* 경로계획

관련 함수:

- `allRouteTargets()`: 전체 경유지와 최종 목표에 화면 번호 부여
- `routeTargets()`: 선택한 반복 번호 범위를 잘라 실행 pose 목록과 각 pose 방향 계산
- `repeatRouteConfig()`: 반복 시작/종료 번호와 0.5~3600초 휴식 값 검증
- `onDrivePointerDown()` / `onDrivePointerMove()`: 최종 목표 위치와 드래그 방향 yaw 동시 지정
- `setWaypointMode()`: 지도 클릭을 경유지 추가/최종 목표 지정 모드로 전환
- `planRouteToGoal()`: 현재 위치에서 경유지와 목표까지 전체 경로 계산
- `runAStar(start, goal, blocked, metrics, soft)`: 진입 금지와 감속 비용을 구분해 A* 실행
- `neighbors(cell, metrics)`: 8방향 이웃 cell 생성
- `heuristic(a, b)`: diagonal distance 기반 휴리스틱
- `reconstructPath()`: 최종 경로 복원
- `BinaryHeap`: open set 우선순위 큐

경로계획 흐름:

1. 현재 위치와 목표를 grid로 변환
2. 벽, 장애물, 분홍색 진입 금지 영역을 blocked set으로 계산
3. 살색 감속 영역에는 높은 이동 비용을 부여
4. 시작점이나 목표점이 막힌 cell이면 실패 처리
5. 경유지별로 A* 실행
6. 계산된 cell 경로를 world 좌표와 `slow` 플래그로 변환
7. 맵 위에 정상 구간은 파란색, 감속 구간은 노란색으로 표시

화면에서 계산한 A*는 Nav2 사용 시 경로 가능 여부와 표시를 담당한다. Nav2 action server가
없으면 전체 경로점이 서버로 전달되어 Pure Pursuit 입력으로 사용된다. 경로 실행 전에는
브라우저의 수동운전 유지 타이머를 먼저 종료하며, 비상주행도 다른 `/cmd_vel` publisher가
있으면 시작하지 않는다.

참고:

- `planPathToGoal()` 내부에는 예전 단일 목표용 코드가 남아 있지만 현재는 맨 앞에서 `planRouteToGoal()`로 위임한다.
- 실제로 사용되는 주 경로계획 함수는 `planRouteToGoal()`이다.

### 그리기

관련 함수:

- `resizeCanvases()`: canvas 크기 갱신
- `startDrawLoop()`: animation frame 기반 draw loop 시작
- `drawMap(canvas, mode)`: 맵 전체 렌더링
- `drawPlanningOverlay()`: grid, inflation, path 표시
- `drawLidarPoints()`: scan 수신 pose와 로봇 yaw를 사용해 LiDAR 로컬 점을 맵 좌표로 변환해 렌더링
- `drawPose()`: 로봇 현재 위치 표시
- `drawEffectiveFootprint()`: footprint와 안전 영역 표시
- `drawObstacleRect()`: 장애물 박스 표시
- `drawWaypointMarkers()`: 경유지 표시
- `drawGoal()`: 목표점 표시

셋업 탭과 주행 탭은 같은 맵 데이터를 쓰지만, mode에 따라 표시하는 overlay가 달라진다.

## `web/index.html`

화면의 정적 구조를 담당한다.

주요 영역:

- 상단 상태/연결 표시
- 셋업 탭
- 주행 탭
- 맵 canvas
- 카메라 영역
- 수동운전 버튼
- 로봇 체크 결과
- 진단 로그 복사 영역

버튼이나 input을 추가할 때는 `id`를 만들고, `web/app.js`의 `bindElements()`와 `bindActions()`에 연결해야 한다.

## `web/styles.css`

화면 스타일을 담당한다.

주요 역할:

- 전체 레이아웃
- 탭 UI
- 맵/카메라 영역 배치
- 수동운전 버튼 배치
- 로봇 체크 결과 색상
- responsive layout

기능 동작은 대부분 `app.js`에 있고, `styles.css`는 표시 방식만 담당한다.

## `config/dashboard_state.json`

현재 셋업과 일부 런타임 값이 저장된다.

주요 영역:

- `setup.map`: 맵 이미지, 해상도, 크기
- `setup.initialPose`: 초기 위치
- `setup.robot`: 로봇 footprint
- `setup.accessory`: 부속품 돌출량
- `setup.safety`: 안전 여유거리
- `setup.object`: 기본 오브젝트 크기
- `setup.obstacles`: 설치된 장애물 목록
- `setup.planner`: grid, 벽 감지, blocked/free cell 설정
- `setup.network`: 서버 IP, 로봇 IP, ROS domain 등
- `setup.robotProfiles`: 로봇별 namespace와 topic 묶음
- `setup.topics`: 현재 활성 topic 묶음
- `runtime`: 현재 위치, 목표, 연결 상태, camera 상태 등

주의:

- 이 파일은 로컬 실행 상태를 담기 때문에 환경마다 달라질 수 있다.
- 인증 정보는 장기적으로 별도 local-only 파일이나 환경변수로 분리하는 것이 좋다.

## 수정할 때 보는 기준

- 화면 버튼 추가: `index.html` → `app.js bindElements()` → `app.js bindActions()` → 필요 시 `server.py do_POST()`
- 새 ROS topic 추가: `DEFAULT_STATE topics` → `topics_for_namespace()` → `RosBridge._init_ros()` → robot check/diagnostics
- 새 셋업값 추가: `DEFAULT_STATE` → `dashboard_state.json` → `fillSetupForm()` → `readSetupForm()` → drawing/planning 함수
- 새 API 추가: `DashboardHandler.do_GET()` 또는 `do_POST()` → `app.js postJson/fetch` 호출
- 경로계획 변경: `blockedCellSet()`, `inflatedCellSet()`, `planRouteToGoal()`, `runAStar()`
- 수동운전 변경: `startManualDrive()`, `stopManualDrive()`, `RosBridge.manual_drive()`, `_manual_watchdog_loop()`
- 로봇 SSH 브링업/점검 변경: `build_dashboard_map_package()`, `build_robot_bringup_script()`, `build_robot_ssh_diagnostics_script()`, `run_ssh_script()`, `RosBridge.robot_bringup()`, `RosBridge.robot_ssh_check()`, `robotBringup()`, `robotSshCheck()`

## 디버깅 순서

수동운전 문제:

1. UI 버전 확인
2. active robot과 topic 확인
3. `/cmd_vel` subscriber count 확인
4. `/cmd_vel` message type 확인
5. `cmd_vel 전달점검`으로 대시보드 발행이 로봇 SBC까지 도착하는지 확인
6. dashboard publish 후 `/odom` 변화 확인
7. 텔레옵키와 같은 topic/type을 쓰는지 비교

카메라 문제:

1. camera on/off 상태 확인
2. raw/compressed topic 확인
3. publisher count 확인
4. frame hz 확인
5. camera node 존재 확인

경로계획 문제:

1. 맵 scale 확인
2. 현재 위치가 맵 안인지 확인
3. 목표와 경유지가 obstacle/inflation 밖인지 확인
4. grid cell size 확인
5. blocked cell과 inflated cell overlay 확인

## 다음 리팩터링 후보

- `server.py`를 `state.py`, `ros_bridge.py`, `http_server.py`, `diagnostics.py`로 분리
- `web/app.js`를 `api.js`, `map.js`, `planner.js`, `drawing.js`, `manual_drive.js`로 분리
- `planPathToGoal()`의 비활성 단일 목표 코드를 정리
- 민감 정보 저장 방식을 환경변수 또는 별도 local config로 분리
- 진단 명령 실행 결과를 profile별로 더 명확히 구조화
