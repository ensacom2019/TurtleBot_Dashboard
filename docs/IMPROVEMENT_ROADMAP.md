# TurtleBot3 Dashboard 개선 방향 문서

## 목적

이 문서는 TurtleBot3 Burger와 ROS2 Jazzy 기반 웹 대시보드의 개선 방향을 정리한다.

현재 목표는 단순히 Nav2에 목표점을 던지는 것이 아니라, 웹 대시보드에서 맵, 로봇 크기, 장애물, 경유지, 센서 상태를 함께 관리하고 안전한 경로를 생성한 뒤 로봇을 제어하는 것이다.

## 현재 기준값

- ROS2 배포판: Jazzy
- 로봇 모델: TurtleBot3 Burger
- 서버 IP: `192.168.20.3`
- 로봇 IP: `192.168.20.7`
- 현재 확인된 ROS_DOMAIN_ID: `1`
- 맵 크기: `180cm x 180cm`
- 터틀봇 footprint: 부속품 포함 `30cm x 20cm`
- 설치 오브젝트 1: `22cm x 20cm`
- 설치 오브젝트 2: `17cm x 10cm`
- 로봇 프로필:
  - `tb3_1`: `/tb3_1` namespace 기준
  - `tb3_2`: root namespace 기준, 예: `/cmd_vel`, `/odom`, `/scan`, `/tf`

기본 개발 설정에는 사용자 요청에 따른 SSH 비밀번호가 들어 있지만, HTTP API와 진단 출력에서는 비밀번호를 숨긴다. 실제 배포 전에는 반드시 비밀번호를 교체하고 별도 secret 저장소로 분리해야 한다.

## 현재 주요 기능

- 셋업 탭
  - 맵 이미지 등록
  - 맵 실제 크기 설정
  - 터틀봇 footprint 설정
  - 장애물 크기와 위치 설정
  - 초기 위치와 방향 설정
  - 로봇 토픽, namespace, ROS domain, 서버 IP 설정
- 주행 탭
  - 맵 위 현재 위치 표시
  - 목표 지점 지정
  - 경유지 지정
  - 수동운전 버튼
  - 로봇 체크
  - 진단 로그 복사
  - 카메라 표시 영역
- ROS 연동
  - `/cmd_vel` 타입 자동 감지
  - `geometry_msgs/msg/Twist`와 `geometry_msgs/msg/TwistStamped` 대응
  - root namespace와 `/tb3_1`, `/tb3_2` namespace 대응
  - `/scan`, `/odom`, `/tf`, `/tf_static`, `/amcl_pose`, camera topic 확인

## 핵심 문제 인식

### 1. Nav2만으로는 충분하지 않음

Nav2는 강력하지만, 맵 품질, footprint, costmap, localization, TF, odom, LiDAR 상태가 조금만 어긋나도 벽에 붙거나 회전만 하거나 목표 이동을 실패할 수 있다.

따라서 대시보드가 자체적으로 다음 정보를 명확히 관리해야 한다.

- 맵의 실제 크기
- 벽과 장애물 위치
- 로봇 footprint
- 안전 여유거리
- 시작 위치
- 목표 지점
- 경유지
- 센서 수신 상태

### 2. `/cmd_vel` 타입과 subscriber 확인이 중요함

텔레옵키로 움직이는데 대시보드 수동운전이 안 되는 경우는 대체로 다음 문제다.

- `/cmd_vel` topic은 맞지만 message type이 다름
- 로봇 드라이버가 `Twist`가 아니라 `TwistStamped`를 구독함
- namespace가 다름
- dashboard가 다른 ROS_DOMAIN_ID에서 실행됨
- subscriber count는 있지만 실제 모터 드라이버가 명령을 처리하지 못함
- 명령은 나가지만 `/odom` 변화가 없음

따라서 수동운전은 단발 publish가 아니라 텔레옵 방식의 지속 publish와 deadman timer가 필요하다.

### 3. 카메라는 topic 이름보다 publisher가 중요함

카메라 topic이 목록에 보여도 실제 publisher가 없으면 화면이 뜨지 않는다.

확인해야 하는 항목은 다음과 같다.

- camera raw topic 존재 여부
- compressed topic 존재 여부
- publisher count
- 마지막 프레임 수신 시간
- `ros2 topic hz` 결과
- camera driver node 존재 여부

## 개선 방향

### 1. 통신 안정화

우선순위가 가장 높다.

- ROS_DOMAIN_ID 표시와 검증
- 서버 IP와 감지 IP 비교
- `ROS_LOCALHOST_ONLY` 확인
- namespace 자동 감지
- `/cmd_vel` 타입 자동 감지
- `/cmd_vel` subscriber count 확인
- `/odom` 변화 감지
- `/tf`와 `/tf_static` 존재 확인
- 로봇 체크 결과를 한 번에 복사 가능하게 유지

### 2. 수동운전 안정화

현재 방향은 텔레옵 방식이 맞다.

- 버튼/키 입력 시 서버가 목표 속도를 저장
- 서버가 약 10Hz로 `/cmd_vel` 지속 발행
- 브라우저는 유지 신호만 주기적으로 전송
- 입력이 끊기면 deadman timer로 자동 정지
- 정지 시 0속도를 여러 번 발행
- 진행 중인 Nav2 goal은 수동 명령이 들어오면 취소

추가 개선:

- 명령 발행 중 `/odom` 변화 없음 감지
- 회전은 되는데 직진이 안 되는 상태 구분
- 직진은 되는데 회전이 안 되는 상태 구분
- 긴급정지 버튼 상단 고정
- 최대 선속도와 각속도 제한값 잠금

### 3. 맵과 장애물 모델 고도화

현재 맵은 `180cm x 180cm` 기준이다. 검은색 선은 벽으로 처리한다.

개선 방향:

- 이미지에서 검은색 벽 자동 추출
- 벽 지우개 기능 개선
- 벽 두께를 grid obstacle로 변환
- 장애물은 크기를 고정하고 위치와 회전만 마우스로 조정
- 터틀봇 footprint를 실제 크기 기준으로 표시
- footprint와 안전 여유거리를 더한 inflation 영역 표시
- 통과 가능 영역과 불가능 영역을 명확히 시각화

### 4. 자체 경로계획 보조

Nav2에 바로 맡기기 전에 대시보드에서 안전 경로를 먼저 계산한다.

추천 구조:

1. 맵을 일정 간격의 grid로 분할
2. 벽과 장애물을 occupied cell로 표시
3. 터틀봇 footprint와 안전 여유거리만큼 obstacle inflation 적용
4. 시작점, 경유지, 목표점을 grid 좌표로 변환
5. A*로 각 구간 경로 생성
6. 경로 smoothing 적용
7. 실제 ROS 목표 pose 또는 저속 주행 명령으로 변환
8. LiDAR로 새 장애물이 감지되면 정지 후 재계산

이 방식은 맵과 물체 크기가 명확한 실내 환경에서 Nav2만 단독 사용하는 것보다 디버깅이 쉽다.

### 5. 위치 추정 신뢰도 표시

맵 위 로봇 위치는 출처를 반드시 표시해야 한다.

신뢰도 순서:

1. AMCL pose + TF
2. `/odom`
3. 사용자가 지정한 초기 위치 + odom anchor
4. preview simulation

화면에는 예를 들어 다음처럼 표시한다.

- `pose source: AMCL`
- `pose source: ODOM`
- `pose source: ANCHOR`
- `pose source: PREVIEW`

초기 위치 지정은 로봇을 물리적으로 이동시키는 기능이 아니다. `/initialpose`는 localization 입력이고, 실제 이동은 목표 이동이나 수동운전으로 수행해야 한다.

### 6. 카메라 기능 개선

카메라 ON/OFF는 명확히 보여야 한다.

개선 방향:

- 카메라 ON/OFF 버튼 고정 표시
- raw/compressed topic 자동 선택
- active camera topic 표시
- publisher count 표시
- 마지막 프레임 시간 표시
- frame rate 확인
- 카메라 브링업 상태 확인
- RealSense D435 확장 대비

RealSense D435를 사용할 경우 추가로 고려할 항목:

- color image
- depth image
- aligned depth
- point cloud
- camera TF
- USB 대역폭
- frame rate와 resolution

### 7. 다중 로봇 지원

로봇이 2대 이상이면 topic 목록만 보는 방식으로는 헷갈리기 쉽다.

개선 방향:

- 로봇 프로필을 명확히 분리
- `tb3_1`, `tb3_2` 각각 topic prefix 저장
- 검색 결과에서 후보 점수 표시
- 후보를 클릭하면 해당 profile에 topic 자동 적용
- 현재 선택된 로봇을 주행 탭에 크게 표시
- 수동운전 명령이 어느 로봇으로 나가는지 항상 표시

### 8. 안전 기능

실물 로봇에서는 안전 기능이 자율주행 기능보다 우선이다.

필수 항목:

- 긴급정지
- deadman timer
- 속도 제한
- LiDAR 전방 최소거리 제한
- 장애물 근접 시 자동 감속
- `/cmd_vel` subscriber 사라지면 자동 정지
- `/odom` 변화 없음 감지
- TF 끊김 감지
- 카메라 frame timeout 감지

## 우선순위 로드맵

### 1단계: 기본 주행 안정화

- 수동운전 텔레옵 방식 유지
- `/cmd_vel` 타입 자동 감지 안정화
- topic namespace 자동 적용
- `/odom` 변화 확인
- 갑작스러운 회전 방지용 stop burst 유지
- 진단 로그 품질 개선

완료 기준:

- 텔레옵키와 대시보드 수동운전이 같은 robot driver를 제어함
- 전진, 후진, 좌회전, 우회전, 정지가 모두 동작함
- 정지 후 제자리 회전이 남지 않음

### 2단계: 카메라와 센서 상태 확정

- 카메라 ON/OFF 버튼 표시
- publisher count 확인
- raw/compressed 자동 선택
- 마지막 프레임 시간 표시
- LiDAR `/scan` hz 확인

완료 기준:

- 카메라 topic이 있을 때 화면이 실제로 갱신됨
- publisher가 없을 때 원인을 화면에서 바로 알 수 있음
- `/scan` 수신 여부와 최근 시간이 표시됨

### 3단계: 셋업 데이터 정확도 개선

- 맵 scale 고정
- 벽 자동 추출
- 장애물 크기 고정 및 위치 이동
- footprint와 inflation 시각화
- grid occupied map 생성

완료 기준:

- 맵 위 모든 물체가 cm 기준으로 배치됨
- 로봇이 지나갈 수 없는 영역이 화면에서 명확히 보임

### 4단계: A* 경로계획

- 시작점, 경유지, 목표점 기반 A*
- grid 장애물 회피
- 경로 smoothing
- 경로 미리보기
- 경로 구간별 상태 표시

완료 기준:

- 사용자가 목표점과 경유지를 지정하면 충돌 없는 경로가 표시됨
- 통과 불가능하면 이유를 표시함

### 5단계: 실제 자율주행 연동

- Nav2 NavigateToPose 사용
- 또는 대시보드 경로를 따라가는 저속 controller 사용
- LiDAR 근접 장애물 감지 시 정지
- 필요 시 경로 재계산

완료 기준:

- 지정된 목표까지 이동
- 중간 장애물 발견 시 멈춤 또는 우회
- 이동 상태와 실패 사유가 화면에 표시됨

## 진단 로그에 포함해야 할 항목

문제 분석용 복사 로그에는 다음 항목이 필요하다.

```bash
ros2 topic list
ros2 node list
ros2 action list
ros2 topic info /cmd_vel -v
ros2 topic info /odom -v
ros2 topic info /scan -v
ros2 topic info /tf -v
ros2 topic info /tf_static -v
ros2 topic info /camera/color/image_raw -v
ros2 topic info /camera/color/image_raw/compressed -v
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /camera/color/image_raw
```

namespace가 있는 로봇은 다음처럼 profile별 topic으로 확인해야 한다.

```bash
ros2 topic info /tb3_1/cmd_vel -v
ros2 topic info /tb3_1/odom -v
ros2 topic info /tb3_1/scan -v
```

## 운영 원칙

- 실물 주행 전에는 반드시 수동운전부터 검증한다.
- 수동운전이 안 되면 자율주행 테스트를 하지 않는다.
- 카메라 topic 이름만 보고 정상으로 판단하지 않는다.
- `/cmd_vel` publisher가 있다고 로봇이 움직인다고 판단하지 않는다.
- 명령 후 `/odom` 변화가 있는지 확인한다.
- 맵 좌표와 실제 cm 기준이 맞는지 먼저 확인한다.
- footprint와 장애물 inflation을 과소평가하지 않는다.

## 결론

이 프로젝트의 핵심 방향은 Nav2 단독 의존이 아니라, 대시보드가 맵/크기/장애물/경유지/센서 상태를 명확히 관리하고 그 데이터를 바탕으로 안전한 주행 명령을 만드는 것이다.

단기적으로는 수동운전과 ROS 통신 안정화가 최우선이다. 이후 맵 grid, obstacle inflation, A* 경로계획, LiDAR 기반 재계산 순서로 확장하는 것이 가장 현실적이다.
