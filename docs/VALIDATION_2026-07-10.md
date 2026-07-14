# TurtleBot Dashboard 코드 검증 결과

기준 버전: `2026-07-13.35`

## 검증 결론

수동주행이 짧은 클릭 직후 바로 0속도로 바뀌던 입력 문제를 수정했다. 현재는 방향 버튼이나 키를 한 번 누르면 유지 신호가 계속 전송되고, 가운데 정지·상단 정지·`Space`·셋업 탭 이동·창 포커스 상실 시 중단된다.

상단 정지 버튼이 서버에 정지를 요청한 뒤에도 브라우저의 기존 유지 타이머가 다시 이동 명령을 보내던 문제도 수정했다. 모든 정지 경로가 먼저 브라우저 유지 타이머를 해제한다.

경유지 이동은 단일 최종 목표에서 실제 동작이 확인된 `NavigateToPose`를 한 지점씩 순차 실행하도록 변경했다. `/navigate_to_pose`가 없을 때만 `NavigateThroughPoses`를 사용한다. LiDAR 직접추종은 A* 점마다 경유지 구간 번호를 전달해 현재 구간 끝을 지나기 전 후반 경로로 건너뛰지 않으며, 최종점이 현재 위치 근처에 있어도 앞선 경유지를 모두 통과하기 전에는 완료 처리하지 않는다. 또한 경로 실행 직전 수동운전 유지 발행을 종료하므로, 남아 있던 `/cmd_vel` 요청이 새 Nav2 목표를 취소하지 않는다.

두 Nav2 action server가 모두 없을 때는 최신 `/scan`, `/odom`, pose와 단일 `/cmd_vel` publisher 조건을 확인한 뒤 LiDAR 비상주행으로 전환한다. A* 경로의 살색 영역은 높은 비용과 저속 구간으로 처리하고, 분홍색 영역은 계속 진입 금지로 유지한다.

## 주요 개선

- `/cmd_vel` subscriber에 맞춰 `Twist`/`TwistStamped` 자동 선택
- subscriber가 아직 발견되지 않은 Jazzy 초기 상태에서는 `TwistStamped` 우선
- 최근 `/odom.header.stamp`와 단조 시계로 `TwistStamped` 로봇 시간 추정
- watchdog을 시스템 시계가 아닌 단조 시계로 변경
- 설정 변경 전 기존 `/cmd_vel`에 정지 명령 발행
- 서버 종료 시 watchdog 종료와 0속도 burst 발행
- 발행 publisher 교체와 watchdog 발행 사이의 lock 적용
- scan/odom publisher가 여러 개인 중복 bringup 경고 추가
- `cmd_vel 전달점검` 추가: 0속도만 발행하고 로봇 SSH에서 수신 확인
- SSH 빠른/상세점검에 로봇 날짜와 NTP 상태 추가
- SSH 비밀번호를 `/api/state`, SSE, 연결/체크 응답에서 제거
- 정적 파일 `../` 경로 탈출 차단
- 비정상 숫자, 과대 JSON/map 업로드, 손상된 카메라 row 데이터 거부
- 경유지 추가/최종 목표 지도 클릭 모드 분리
- 최종 목표가 비어 있을 때 `(0, 0)`으로 잘못 이동하던 입력 차단
- Jazzy `/navigate_through_poses` 액션과 로봇별 namespace 설정 추가
- 로봇 체크와 SSH 진단에 `NavigateThroughPoses` action server 확인 추가
- LaserScan ranges/angles 유효성 검사와 최대 720점 다운샘플링
- Pure Pursuit 기반 A* 선 추종과 최종 방향 정렬
- 미래 직사각형 footprint를 0.1초 간격으로 투영하는 충돌 예측
- 살색 감속 구간과 LiDAR 장애물 거리 기반 속도 배율 적용
- scan/odom/pose timeout, cmd_vel subscriber, 단일 publisher 시작 조건
- 충돌 예상 시 정지 대기 후 장애물이 사라지면 자동 재개
- Inflation 폭을 0.00~0.40m 슬라이더/숫자로 실시간 조정하고 0m에서 살색 영역만 비활성화
- 분홍 금지 반경을 0.05~0.40m에서 조절하고 A*와 지도에 즉시 반영
- fallback 시작 순간 scan/odom이 일시적으로 stale이면 최대 2.5초 동안 새 샘플을 기다린 뒤 자동 재검사
- 주행 중 Scan/Odom 안전 정지 제한을 각각 0.5~1.0초 슬라이더와 숫자 입력으로 조절
- 편집 중 SSE가 로컬 오브젝트·벽 setup을 이전 서버 값으로 덮어쓰지 않도록 setup/runtime 병합 분리
- 2.5초보다 긴 센서 공백에서도 목표를 최대 15초간 큐에 유지하고 복구 즉시 자동 출발
- LiDAR 충돌 예상 시 정지·안전 방향 회전·저속 전진 탈출·후방 확인 후 저속 후진·기존 A* 경로 재합류
- `/scan` 표시점을 최대 240개·5Hz로 제한해 셋업/주행 맵에 청록색 점으로 표시
- 세션별 주행 JSONL 저장, Markdown 복사·다운로드, 비밀번호 마스킹과 용량 제한
- 주행 탭에서 Nav2를 건너뛰고 LiDAR A* 직접 추종을 강제하는 테스트 토글
- 현재 `Sprite-1.png` 맵, 초기 위치와 장애물 2개를 코드 기본 설정으로 동기화
- 서버 시작·상태 조회·연결 새로고침 시 로봇 서브넷과 운영체제 라우팅 기준으로 서버 IPv4 자동 설정
- SSH 브링업에서 `/dev/ttyACM1` 우선 OpenCR 식별, Arduino 거부, `/dev/ttyACM0` 검증 폴백과 실행 후 노드 확인

## 실행한 검증

```text
Python compile                    통과
JavaScript syntax                 통과
dashboard_state.json parse        통과
bringup/SSH/cmd_vel shell syntax  통과
Python unittest 38개              통과
API state/manual/route/path smoke 통과
데스크톱 UI                       통과
390x844 모바일 UI                통과
브라우저 console warning/error    없음
```

`/api/run_logs`에서 수동 명령과 정지 이벤트를 확인했고 `/api/run_logs/download`가 UTF-8 Markdown 첨부 파일을 반환하는 것을 검증했다. 주행 탭의 세 버튼 표시, 세션 요약 갱신, 로그 복사 완료와 실제 클립보드 내용을 브라우저에서 확인했으며 클립보드 권한 응답이 멈추는 환경에서는 1초 뒤 선택 복사 방식으로 전환된다.

프리뷰에서 `전진`을 한 번 클릭한 뒤 0.9초가 지나도 `v=0.08`과 활성 표시가 유지되는 것을 확인했다. 상단 `정지` 클릭 후에는 유지 표시가 해제되고 상태가 `stopped`로 바뀌는 것도 확인했다.

주행 화면에서 경유지 2개와 최종 목표 1개를 지도에 지정해 A* 3지점 경로가 생성되고, `/api/route` 전송 후 각 지점을 거쳐 `succeeded`가 되는 것을 확인했다. 최종 목표 없이 경유지만 만든 경우에는 전송하지 않고 최종 목표 모드로 전환되는 것도 확인했다.

새 A* 계획에서 55개 경로점 중 25개가 감속 구간으로 표시되고 서버가 55개 경로점을 모두 수신하는 것을 확인했다. LaserScan 필터, 정상 공간 속도 유지, 예상 footprint 충돌 정지, stale scan 시작 거부, 신선한 센서 조건 시작 승인과 목표 완료 정지를 단위 테스트로 검증했다.

fallback 요청 시 센서가 아직 없는 상태에서 50ms 뒤 scan/odom/pose가 들어오는 조건을 재현해, 요청이 즉시 실패하지 않고 새 샘플을 기다린 뒤 정상 승인되는 것을 검증했다. 제한 시간 이후에도 센서가 없으면 오류에 실제 센서 나이와 설정 제한값을 함께 표시한다.

초기 2.5초 대기 이후 목표가 `lidar_fallback_waiting`으로 큐에 들어간 상태에서 센서 샘플을 복구해 `fallback_starting`으로 자동 전환되는 것을 단위 테스트로 확인했다.

Inflation 폭을 `0.20m`로 설정했을 때 감속 셀이 45개, `0.00m`로 설정했을 때 0개로 즉시 다시 계산되는 것을 확인했다. 슬라이더와 숫자 입력은 양방향으로 동기화됐고 모바일 390x844 화면에서도 가로 넘침이 없었다.

분홍 금지 반경은 기본 `0.05m`, Inflation 폭은 기본 `0.10m`로 변경했다. 숫자 입력과 슬라이더가 함께 바뀌며 분홍 반경은 그 아래 값이 `0.05m`로 보정된다. LiDAR 직사각형 footprint 충돌 정지는 이 A* 표시 반경과 별도로 유지된다.

Scan/Odom 제한은 각각 기본 `0.6초`로 표시되고 슬라이더와 숫자가 함께 바뀌는 것을 확인했다. Scan `0.4초` 입력은 `0.5초`, Odom `1.1초` 입력은 `1.0초`로 보정됐으며 서버 설정 정규화도 같은 범위를 적용한다.

오브젝트 편집 후 SSE를 1.6초 이상 수신한 다음 저장해도 새 장애물 3개가 서버 setup에 그대로 저장되는 것을 확인했다. 검증용 장애물은 삭제하고 기존 2개 상태로 복구했다.

LiDAR 회복 로직은 좌우 중 더 넓은 방향 선택, 전방 장애물에서 후진 허용, 후방 1cm 여유 장애물에서 후진 금지, 최초 0.25초 정지 후 회전 전환을 단위 테스트로 검증했다. footprint가 이미 금지 경계와 겹친 경우에도 겹침을 키우지 않고 최종 여유가 증가하는 후진·회전·전진은 허용하며, 안전 후보가 없는 `fallback_recovery_trapped` 상태는 0.75초마다 최신 scan으로 재검사한다.

몸체 치수는 `0.26 x 0.16m`, 외접 표시 반경은 `0.153m`로 설정했다. 오브젝트 1은 `0.16 x 0.16m`, 오브젝트 2는 `0.16 x 0.10m`로 기본값·현재 장애물·프리셋을 동기화했다.

현재 저장된 초기 pose `0.188, 0.224, 1.571rad`와 장애물 중심 좌표 `1.027, 0.449`, `1.218, 1.342`를 코드 기본값으로 동기화했다.

720개 LiDAR 점을 표시용 240개로 제한하고 3자리 좌표로 변환하는 단위 테스트를 추가했다. 브라우저에서 `LiDAR 점 표시` 토글의 저장·복원을 확인했으며, 설정 저장 후 JSON에 `lidarPoints`와 `lidarPose`가 남지 않는 것도 확인했다. 실제 점 배치는 ROS `/scan`이 없는 Windows 프리뷰에서는 검증할 수 없어 로봇 환경 확인이 남아 있다.

주행 로그는 세션별 JSONL 파일과 최근 4,000개 이벤트 메모리 버퍼로 분리했다. 비밀번호/secret 마스킹, Markdown+JSONL 리포트 생성, 초기화 시 새 세션 생성은 단위 테스트로 검증했다. 센서 요약은 1 Hz, LiDAR 비상주행 제어 상태는 2 Hz로 기록하며 목표·경유지·수동 명령·취소·정지는 즉시 기록한다.

강제 LiDAR 직접추종 요청은 Nav2 Action 가용성 함수를 호출하지 않고 `_start_lidar_fallback()`으로 바로 들어가며, 센서가 stale이면 목표 대기열을 사용하는 것을 단위 테스트로 검증했다.

회전 완료 후 전방 0.8초 예측 궤적이 안전하면 `forward` 복구 단계로 전환되고, 전방 1cm 여유에 장애물이 있으면 전진 후보를 거부하는 것을 단위 테스트로 확인했다.

## 실물 로봇 확인 순서

1. 서버 PC에서 `./run_ubuntu.sh`를 실행한다.
2. 셋업 탭에서 `ROS_DOMAIN_ID=1`, `ROS_LOCALHOST_ONLY=0`, `/cmd_vel`, `/odom` 설정을 저장한다.
3. `로봇 체크`에서 `/odom` publisher와 `/cmd_vel` subscriber/type을 확인한다.
4. `cmd_vel 전달점검`을 누른다. 이 단계는 0속도만 사용하므로 로봇을 움직이지 않는다.
5. 결과가 성공이면 주행 탭에서 선속도 `0.05~0.08 m/s`로 전진을 한 번 누른 뒤 즉시 정지 동작을 확인한다.
6. `/odom` 위치와 맵 표시가 함께 변하는지 확인한다.
7. `Nav2 NavigateToPose`가 `available`인지 확인한 뒤 경유지, 최종 목표 순서로 찍고 `경로 실행`을 누른다. 상태가 `Sequential route started: 1/N`으로 시작해 지점 번호가 순서대로 증가하는지 확인한다.
8. 비상주행 검수 시에는 Nav2와 teleop publisher를 종료하고 `로봇 체크`의 `LiDAR 비상주행`이 통과하는지 확인한다.
9. 바퀴를 들거나 넓은 빈 공간에서 기본 속도 `0.03 m/s`로 시작해 정지·감속·센서 케이블 분리 시 0속도 동작을 먼저 확인한다.

`cmd_vel 전달점검`이 실패하면 ROS domain, RMW, 방화벽, topic/type 문제다. 전달점검은 성공하지만 모터가 움직이지 않으면 `turtlebot3_node`, OpenCR 연결, 모터 전원 쪽을 확인한다. `/scan` publisher가 2개 이상이면 중복 `robot.launch.py`부터 한 세트로 정리한다.

## 남은 검증 한계

현재 작업 환경은 Windows 프리뷰라 실제 TurtleBot3/OpenCR, ROS2 Jazzy DDS, 카메라 스트림과 LiDAR 비상주행의 물리 동작은 직접 검증하지 못했다. LiDAR 평면보다 낮은 물체, 유리, 흡광 재질과 센서 사각은 감지하지 못할 수 있으므로 위 실물 검수가 필요하다.

Jazzy의 ros2_control `diff_drive_controller`는 stamped velocity와 command timeout을 사용한다. 공식 TurtleBot3 Jazzy 노드는 `Twist`와 `TwistStamped`를 모두 받을 수 있으며 속도값을 OpenCR로 전달한다.

- [ROS2 Control Jazzy diff_drive_controller](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [TurtleBot3 Jazzy turtlebot3_node source](https://github.com/ROBOTIS-GIT/turtlebot3/blob/jazzy/turtlebot3_node/src/turtlebot3.cpp)
- [Nav2 Jazzy NavigateThroughPoses action](https://api.nav2.org/actions/jazzy/navigatethroughposes.html)
- [Nav2 Inflation Layer](https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html)
- [Nav2 Regulated Pure Pursuit](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)
- [ROS2 Jazzy sensor_msgs](https://docs.ros.org/en/jazzy/p/sensor_msgs/__message_definitions.html)
