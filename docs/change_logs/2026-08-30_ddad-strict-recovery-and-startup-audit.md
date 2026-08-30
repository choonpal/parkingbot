# 2026-08-30 ddad strict 복구·기동 부하 감사

## 결론

- 복구 기준: `ddad9ac`
- 작업 branch: `recovery/ddad-strict-20260830`
- 기능 코드 기준점: `a5e7290` (이 문서 커밋은 그 위에 추가)
- 현재 실차 판정: **전체 production 기동 NO-GO, 차량 모션 NO-GO**
- 현재 장비 상태: Jetson, Front, Rear production stack 모두 종료
- 이번 작업 중 차량 이동: 없음

`ddad9ac`는 자잘한 문제가 남아 있었지만 기본 강체 주행 경로가 작동했던 마지막
기준으로 선택했다. 이후 변경을 통째로 신뢰하지 않고 heartbeat, 운용 도구,
TensorRT와 mission perception을 작은 커밋으로 다시 이식했다.

Jetson의 두 TensorRT 엔진을 동시에 cold-load하던 메모리 피크는 cam2 시작을
15초 늦추어 해결했다. 실제 기동에서 cam0와 cam2가 모두 정상 로드됐고 기존의
NvMap/NVML/OOM 오류는 재발하지 않았다. 하지만 Front/Rear RPi에서 다른 ROS
프로세스를 cold-start할 때 Python/USB 스케줄링 지연이 300ms heartbeat watchdog을
침범하는 별도 문제가 남아 있다. 브리지 단독 시험은 양쪽 모두 통과했지만 전체
기동은 두 번 모두 FAULT로 끝났으므로 이 상태에서 자동 진입을 실행하지 않는다.

## 커밋 구성

| 커밋 | 변경 | 현재 판정 |
|---|---|---|
| `ddad9ac` | 복구 기준 | 기본 강체 경로의 마지막 기준점 |
| `ea63aef` | heartbeat 생성과 UART write scheduler를 ROS callback 부하에서 분리 | 정적·단독 통과, 전체 기동 부하에는 불충분 |
| `55a2e05` | strict local workspace 배치를 production 도구가 인식 | 로컬 strict overlay 기동에 사용 |
| `34ded5b` | bridge executor를 single-thread로 제한 | callback 경쟁 감소, 전체 cold-start 문제는 잔존 |
| `e4c7197` | mission target snapshot, YOLO unload/reload, cam2 순차 시작 | TensorRT 순차 로딩 실차 확인; snapshot 전환 실차 미확인 |
| `a5e7290` | UART RX 전용 thread, Rear persistent camera path | 브리지 단독 통과; 전체 cold-start 실패 |

원래 작업 트리 `${HOME}/parkingbot`은 수정하지 않았다. 모든 변경은
`${HOME}/parkingbot_strict_ddad`의 복구 branch에서 수행했다.

## TensorRT와 mission perception 변경

### cold-start 순차화

`cctv_server_dual.launch.py`는 cam0 YOLO를 즉시 시작하고 cam2 YOLO를 기본
15초 뒤 시작한다. GPU inference guard도 유지하므로 두 프로세스의 CUDA 초기화와
추론 section은 직렬화된다.

실차 로그 `20260830_011911`에서 확인한 순서는 다음과 같다.

- cam0 engine load 성공
- 15초 지연 뒤 cam2 engine load 성공
- cam0/cam2 detection envelope 수신 뒤 `PERCEPTION_AVAILABLE`
- 종전 NvMap error, NVML internal assert, model-load OOM 재발 없음

### 정차 타깃 확정 뒤 YOLO 해제

YOLO는 첫 검출 즉시 내려가지 않는다. 다음 조건이 모두 성립해야 mission snapshot을
소유한다.

1. 대기영역 차량이 stationary hold를 통과해 target latch됨
2. segmentation mask 기반 차량 길이·폭이 유효함
3. Fleet가 운영 승인 뒤 `WAIT_LIFT`로 진입함

snapshot에는 target, vehicle spec, merged obstacle/map 입력, 빈 슬롯 상태가
포함된다. 이후 `cctv_merge`는 snapshot을 fresh timestamp로 계속 발행하고
`/parking/perception_suspend=true`를 보낸다. 두 YOLO 노드는 모델 참조와 classifier를
해제하고 CUDA cache를 반환한다. mission complete 뒤에는 GPU guard 아래에서 모델을
다시 순차 로드한다.

다음 경로는 중지하지 않는다.

- 듀얼 CCTV 원본·rectified 영상
- `cctv_robot_marker`의 Front ID2, Rear ID1 상판 위치추정
- Rear 전방 카메라의 상대 ID0 추적
- wheel encoder odometry, pose fusion, 강체 예측·feedback
- STM32 hardware watchdog과 0속도 safety path

이 snapshot/unload 전환은 회귀 테스트를 통과했지만, 전체 로봇 기동이 먼저
FAULT로 막혀 실제 `WAIT_LIFT`까지 도달하지 못했다. 따라서 **구현 완료, 실차
mission 전환 미검증**으로 기록한다.

## UART/heartbeat 변경과 결과

heartbeat TX는 전용 producer thread와 단일 UART writer를 사용한다. `a5e7290`은
ACK와 telemetry RX도 100Hz 비차단 전용 thread로 옮겼다. ACK가 이미 kernel UART
buffer에 있는데 ROS timer가 지연돼 host가 timeout을 먼저 선언하는 경로를 제거한
것이다. 300ms watchdog이나 ACK timeout은 완화하지 않았다.

Rear 전방 카메라는 숫자 index 대신 설정의 persistent path를 launch까지 전달한다.

```text
/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Autodarts_DIY_Cam_SN0001-video-index0
  -> /dev/video0
```

### 통과한 시험

| 시험 | 결과 |
|---|---|
| vision/merge/GPU focused regression | `56 passed` |
| UART scheduler/bridge/ops focused regression | `67 passed, 1 skipped` |
| strict overlay 증분 colcon build | PASS |
| Front bridge-only 15초 HELLO/HB/zero | PASS, heartbeat 경고 없음 |
| Rear bridge-only 15초 HELLO/HB/zero | PASS, heartbeat 경고 없음 |

bridge-only 시험은 속도 command를 발행하지 않았고 종료할 때 0속도를 전송했다.

### 실패한 전체 기동

#### `20260830_011911`

- TensorRT cam0/cam2 순차 로딩 성공
- Rear가 다른 ROS 노드를 cold-start하던 중 최초 HB TX gap 약 `170.4ms`
- Rear `HEARTBEAT_ACK_TIMEOUT` 뒤 UART write timeout과 session recovery 연쇄
- 당시 Rear camera는 숫자 `camera_id=0` open 실패
- production stack 종료, 차량 이동 없음

#### `20260830_012720`

- UART RX thread와 persistent Rear camera launch 반영 뒤 재시도
- Front가 ultrasonic node cold-start 근처에서 HB TX gap `138.6ms` 경고 후
  `HEARTBEAT_ACK_TIMEOUT`, 이어 UART write timeout
- 이후 Front/Rear 모두 수백 ms~수초 TX gap과 recovery가 연쇄
- 사용자 관찰상 전체 준비 중 SSH 연결도 불안정해짐
- production stack 종료, 차량 이동 없음

두 결과는 TensorRT 문제가 해결됐다고 RPi heartbeat 문제까지 해결되는 것은
아님을 보여 준다. 장애가 다른 Python 프로세스의 cold import 시점과 반복해서
겹치므로 CPU/USB scheduling pressure가 강한 원인 후보지만, 아직 단일 원인으로
확정하지 않는다.

## 배포 상태

active 재배포 전 원격 Front/Rear Git HEAD는 `ddad9ac`이고 필요한 Python/launch
파일만 overlay 복사된 상태였다. 따라서 당시 controller `a5e7290`과 원격 Git
SHA가 같은 clean 배포가 아니었다. 이 상태는 아래 Camera 및 workspace 경로
재감사에서 `parkingbot_active` release로 교체했다.

확인한 배포 SHA-256:

```text
3417c843439872bc0243298f5efbfcf5cb375c052f07698e3b9973b7c3fed617  stm32_bridge_node.py (Front/Rear)
7770886ff833bc1719aafc47c6edd0afcd85da98e772e360f862b44c1d7cbd68  rear_robot.launch.py (Rear)
```

Rear 복사 중 패키지 루트에 잘못 놓였던 두 임시 파일은 실행 경로에서 제거하고
삭제하지 않은 채 아래 위치로 이동했다.

```text
/tmp/parkingbot-deploy-stray-a5e7290-stm32_bridge_node.py
/tmp/parkingbot-deploy-stray-a5e7290-rear_robot.launch.py
```

### Camera 및 workspace 경로 재감사

2026-08-30 후속 감사에서 `/dev`를 제한된 실행 환경 안에서 확인하면 Jetson
카메라가 없는 것처럼 보이는 false negative가 재현됐다. 실제 host에서 다시
확인한 결과 CAM0 C922, CAM2 C920, Front/Rear 각 Sonix camera는 정상이며 장비별
영구 경로는 `REAL_ROBOT_DEPLOYMENT_RUNBOOK.md`에 고정했다. 같은 Sonix by-id가
Front와 Rear에 모두 존재하므로 camera path는 host와 분리해서 기록하면 안 된다.

또한 설정은 `parkingbot_main_0a52285`를 가리키지만 일부 workspace는 Git 저장소가
아니고 strict 파일이 선택적으로 overlay되어, 디렉터리 이름·Git revision·실행
Python 파일이 일치하지 않는 상태였다. 다음의 작은 fail-closed 조치를 추가했다.

- 운영 설정은 host-local `parkingbot_active` symlink만 사용
- package-only 배포는 `.parkingbot_revision`으로 배포 SHA 기록
- `robot_doctor`와 start 전 preflight에서 ROS package prefix와 Python module의
  realpath가 선택 workspace 아래인지 확인
- CAM0/CAM2와 내부 Rear camera는 `/dev/v4l/by-id`만 허용하고 Rear camera는
  Rear 호스트에서 존재 여부 확인
- 설치된 `robotctl_core`는 `CONTROL_WORKSPACE`를 우선해 저장소를 찾고 예전
  `~/parkingbot`을 우연히 선택하지 않음

이 조치는 device나 workspace를 자동 추측하지 않는다. 명시한 active target과
host-local persistent path가 맞는지만 빠르게 검증한다.

### Active 배포 후 빠른 단계 시험

세 장비에 `parkingbot_active`를 만들고 Front/Rear package-only release에는
`89081ad` revision marker를 기록했다. Jetson model도 과거 `ros2_ws`가 아니라
active install/share 경로를 사용하도록 현장 설정을 바꿨다. 최종
`robot_doctor`는 `READY`였다.

정적·무주행 시험 결과는 다음과 같다.

- Front/Rear bridge-only: HELLO v2, servo attach, hardware ready, heartbeat 정상
- Jetson CAM0 C922와 CAM2 C920 각 단독: 640x360, reported 30fps, open/read 정상
- CAM0+YOLO0, CAM2+YOLO2 각 단독: TensorRT, homography, coverage/detection 처리 정상
- shared DDS Rear bridge → Front bridge → CAM0 → CAM2 → YOLO0 순차 join:
  heartbeat timeout, UART write, duplicate ACK 경고 없음
- 차량 이동과 `cmd_vel` publisher 없음; 종료 때 0속도 전송 확인

반면 production dual launch의 `yolo_cam2_start_delay_s=15` 시험에서는 두 번째
engine warm-up 중 cam0 detection이 약 4초 stale이 됐고 `NvMapMemAlloc error 12`가
2회 발생했다. 이후 cam0/cam2 모두 `PERCEPTION_RECOVERED`로 돌아왔지만 peak는
RAM 약 4.95/7.62GB, swap 3MB, CPU 50~77%, GR3D 최대 89%였다. 따라서 단독
카메라나 DDS 영상 publisher 자체는 RPi heartbeat 원인으로 보이지 않지만, dual
TensorRT cold-load/상시 운용에는 충분한 여유가 있다고 볼 수 없다.

후속 변경에서는 카메라별 YOLO 프로세스 2개를 제거하고, 한 프로세스 안에서
TensorRT/Ultralytics 모델 객체 1개를 cam0/cam2가 공유하도록 바꿨다. 카메라별
homography, coverage, sequence와 detection topic은 독립 상태로 유지하고, 최신
프레임 round-robin scheduler가 총 추론률을 제한한다. 총 10Hz 무구동 시험에서
두 detection topic은 각각 약 5Hz였고, 70초 동안 perception 재중단이나
`NvMapMemAlloc` 오류는 없었다. RAM은 약 4.27/7.62GB로 기존 dual-process peak
약 4.95GB보다 낮았다. CPU 사용률이 높아 production 기본 총 추론률은 6Hz로
설정했다. mission snapshot 때는 shared engine을 삭제·재로드하지 않고 추론만
pause/resume한다.

현재 unload 계약은 target 정차와 치수 확정만으로 실행되지 않는다. Fleet가 운영
승인 뒤 `WAIT_LIFT`에 들어가야 mission snapshot이 active가 되고
`/parking/perception_suspend=true`가 발행된다. 이번 시험은 PARK/승인을 보내지
않았으므로 suspend 미발행은 계약상 정상이다. 다만 승인 전 대기 중에도 위 메모리
압력이 유지되므로, "정차+치수 확정 시 더 일찍 snapshot/unload" 정책은 별도
안전 판단이 필요한 다음 개선 후보로 남긴다.

### shared YOLO 후 전체 Jetson 부하 제한

15fps 전체 구성에서도 수정 전 Jetson CPU는 코어별 약 87~96%였다. 빠른 분리
시험에서 카메라·보정·merge만으로 약 30~57%, shared YOLO 추가 시 약 40~82%,
Production ArUco까지 추가하면 약 52~92%였다. 따라서 두 YOLO 프로세스만의
문제가 아니라 rectification, merge/map JPEG, ArUco와 관제 렌더링이 함께 CPU를
소모한다고 판단했다.

구조 변경 없이 다음 상한만 적용했다.

- CCTV 입력 15fps
- shared YOLO cam0+cam2 총 4Hz
- merge/map 5Hz
- Production CCTV ArUco는 카메라별 두 프레임 중 하나만 처리
- 관제탑은 Production ArUco를 중복 실행하지 않음
- 관제 영상 5fps, BEV 2fps
- debug overlay가 꺼진 Kiosk는 카메라 JPEG worker를 만들지 않음

두 UI가 실제 접속된 전체 무주행 재시험에서 코어별 약 47~69%, RAM 약
4.83/7.62GB였고 detection, marker, merge와 HTTP 5000/5008 응답은 유지됐다.
Front heartbeat는 9,327 ACK 동안 loss 0이었다. 같은 시점 Rear는 heartbeat
fault가 아니라 Wi-Fi 자체에서 이탈해 ping/SSH/ROS publisher가 모두 사라졌으므로,
Rear 재연결 뒤 동일 SHA 배포와 heartbeat 확인이 남아 있다.

시간 제한으로 process를 종료할 때 bridge UART reader와 일부 rclpy node가 이미
닫힌 context에 publish하며 traceback을 남기는 shutdown race도 확인했다. 실행 중
heartbeat 장애와는 구분되며 모든 camera/serial 점유는 시험 뒤 해제됐다.

## 다음 작업과 운용 제한

전체 `robotctl start`를 반복하지 않는다. 특히 SSH가 불안정해지는 상태에서 세
장비 전체 준비를 다시 시도하지 않는다.

다음 순서는 아래처럼 제한한다.

1. Jetson을 올리지 않고 RPi 한 대만 시험
2. bridge-only 통과 뒤 state machine, ultrasonic, pose fusion, individual move,
   camera/ArUco를 한 번에 하나씩 긴 간격으로 cold-start
3. 최초로 heartbeat gap 또는 UART write timeout을 만드는 프로세스·자원 사용량 기록
4. 필요하면 비-bridge 노드의 nice/CPU affinity 또는 더 긴 launch stagger를 적용
5. 300ms firmware watchdog과 host ACK timeout은 완화하지 않음
6. 양쪽 단독 조합 통과 후에만 movement 없는 전체 준비를 한 번 수행
7. 그 뒤에도 `stop_after_align=true` 범위의 자동 진입·정렬만 검토

현재는 encoder 기반 예측, 강체 feedback, mission snapshot 업그레이드를 제거할
이유가 없다. 다만 전체 cold-start gate를 통과하기 전에는 이 기능들의 실차 통합
성공을 주장하거나 차량 모션에 사용하지 않는다.
