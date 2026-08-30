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

### Front 바퀴 공중 pulse

Front 재부팅 뒤 bridge-only로 HELLO/servo attach/hardware ready를 다시 확인하고
바퀴를 띄운 상태에서 `drive_pulse`를 실행했다. 첫 pulse는 도구가 공통
`base_link` frame을 보내 역할별 `front_base`만 허용하는 bridge가 전부 거부했다.
도구를 `{role}_base`로 수정하고 회귀 `22 passed` 뒤 재시험했다.

- 0.0628m/s x 0.8s 정방향: wheel odom +4.39cm, 횡 -0.24mm
- 같은 역방향: 누적 x +0.83mm, 횡 오차는 사실상 0
- 매 pulse 뒤 manual false, 네 모터 RPM/PWM 0
- 최종 heartbeat ACK 3,606, loss 0, recovery 없음

이 시험은 바퀴 공중 무부하 결과이며 바닥 거리 보정이나 하중 주행 통과를
뜻하지 않는다.

시간 제한으로 process를 종료할 때 bridge UART reader와 일부 rclpy node가 이미
닫힌 context에 publish하며 traceback을 남기는 shutdown race도 확인했다. 실행 중
heartbeat 장애와는 구분되며 모든 camera/serial 점유는 시험 뒤 해제됐다.

### Front 3축 잭업·빈 바닥 저속 재확인

Front bridge-only와 최신 `drive_pulse`를 domain 42에서 사용했다. 초음파
offset 수동 왕복은 2026-08-29에 이미 완료했으므로 반복하지 않았다.

공중 0.4초 pulse 중 첫 `+Y`는 node가 bridge DDS discovery 전에 시간을
소진해 명령의 일부만 전달됐고, 다음 `-Y`는 manual enable ACK가 pulse
종료 뒤 도착해 속도가 전혀 전달되지 않았다. 모터 불량이 아니라
시험 producer 시작 순서 문제였다. `drive_pulse`가 `/front/manual_active=true`
ACK와 command/enable subscriber를 모두 확인한 뒤 실제 pulse 타이머를 시작하고,
5초 안에 확인하지 못하면 속도 명령 없이 중단하도록 수정했다. 대상
회귀는 `23 passed`였다.

수정 뒤 공중에서 횡이동·회전 양방향 부호를 확인했고, 회전 왕복 뒤 yaw
잔차는 약 `+0.33°`였다. Front를 빈 바닥에 내린 뒤 동일한 저속
조건으로 다음을 확인했다.

- 전진 `+24.96mm`, 후진 `-22.04mm`; X 왕복 잔차 `+2.92mm`
- 횡이동 `+21.23mm`, 반대 방향 `-15.85mm`; Y 왕복 잔차 `+5.38mm`
- 회전 `+3.20°`, 반대 방향 복귀; yaw 왕복 잔차 약 `+0.33°`
- 전체 누적 위치 잔차 X 약 `+3.3mm`, Y 약 `+5.5mm`
- 종료 후 manual false, FL/FR/RL/RR RPM/PWM 모두 0, hardware ready true
- heartbeat ACK 13,475, loss/timeout/UART write failure 0, recovery/fault 없음

바닥 횡이동의 양방향 거리 차이는 현재의 짧은 무하중 pulse에서 바닥 마찰과
미끄럼의 영향을 받은 결과로, 부호·정지 gate는 PASS이지만 자동 차선 보정
정밀도를 증명하지는 않는다.

### DISARMED 기동 정책 배포와 최초 PARK 중단

Front/Rear에 `e71fe80`을 package-only release로 배포하고
`parkingbot_active`를 해당 release로 전환했다. bridge, state machine,
ultrasonic, pose fusion을 함께 cold-start하자 두 RPi 모두 약 30초 동안
ping/SSH/ROS가 끊기고 UART write/heartbeat timeout이 발생했다. 새 정책은 이
구간을 DISARMED recovery로 처리해 mission fault 고착과 예기치 않은 구동은
막았다. Front는 자동 복구했고 Rear는 기존 recovery session에서
`STM32 ERR: HEARTBEAT_TIMEOUT`을 반복해, 나머지 노드는 유지한 채 bridge만 새
HELLO session으로 재시작한 뒤 `hardware_ready=true`를 회복했다. 즉 기동 중
fault 격리는 검증됐지만 cold-start 부하와 recovery 정체 원인은 해결되지 않았다.

PARK 최초 요청은 Fleet 내부에 과거 `motion_fault` 문자열이 남아 있어
`ROBOT_NOT_IDLE`로 거절됐다. UI는 최근 fault만 표시해 `fault=null`이었지만 Fleet는
수명 전체의 마지막 non-empty fault를 보존했다. 현재 IDLE/READY와 활성 fault
없음을 확인하고 운영자 승인 뒤 과거 값을 초기화하자 두 번째 요청은 승인됐다.
임시 토픽 발행에 의존하지 않는 명시적 reset/rearm 계약이 필요하다.

승인된 mission에서 Front는 `TO_REAR_STAGING`, Rear는
`WAIT_FRONT_STAGED`에 진입했다. 당시 고정된 차량 target은
`(0.517, 0.627, yaw=148.2deg)`였고 코드가 계산한 Front staging goal은
약 `(1.239, 0.179)`였다. 그러나 CCTV 기준 Front는 초기
`(3.600, 0.885, -89.2deg)`에서 `(2.435, 2.248, -89.1deg)`로 이동해 계산
goal의 Y 방향과 반대로 벗어났다. 60초 뒤 `TO_REAR_STAGING_TIMEOUT`이 발생했고,
운영자 관측 직후 `/emergency_stop=true`를 발행해 양쪽을 FAULT/ESTOP 상태로
고정했다. 차량 PCA yaw 오류만으로 실제 반대 방향 이동을 설명할 수 없으므로
Front CCTV heading의 180도 모호성, marker 장착 offset과 map/body 변환을 짧은
단축 시험으로 구분하기 전에는 PARK를 재시도하지 않는다.

같은 정지 상태에서 production CCTV pose는 Front ID2와 Rear ID1을 모두 fresh
`visible=true`로 보고했고 `/front|rear/cctv_pose`도 발행했다. 반면 Control Tower
5008은 중복 CPU 사용을 막기 위해 자체 `enable_aruco=false`인데도 guidance와
marker overlay를 자체 detector의 빈 `markers[]`에서 계산해 "마커 미검출"로
표시했다. Rear 전면 카메라의 ID0 `/sync/marker_visible=false`는 실제 미검출이다.
UI는 detector를 다시 켜지 말고 production pose/visible 토픽으로 ID2/ID1을
그리며 ID0 상태를 별도 표시해야 한다.

### 진단 observer를 끈 staging 실차 재시험

전체 stack 기동 직후 startup observer가 살아 있는 동안 Front/Rear에서
UART/heartbeat 오류가 반복됐고, observer 종료 후에는 35초 동안 통신 fault가
0건이었다. Rear camera를 별도로 재기동하며 DDS participant가 합류했을 때 양쪽에
짧은 통신 오류가 한 번 발생했지만 DISARMED 상태에서 자동 복구됐고, camera가
안정된 뒤 추가 32초 동안 fault는 없었다. 따라서 전체 진단 observer의 반복
join/leave는 RPi 부하를 키우는 요인이며 주행 중 상시 실행 대상이 아니다.

이 상태에서 PARK를 한 번 승인하자 Front는 약 2.13m staging 경로를 약 67초 동안
주행했고 UART write/heartbeat failure는 0건이었다. Rear는 정지 상태로
`WAIT_FRONT_STAGED`를 기다리다가 기존 공통 60초 제한으로 fault가 났고, Front는
그로부터 약 7.2초 뒤 `READY_TO_SCAN`에 도착했다. 즉 이번 중단의 직접 원인은
주행 통신이 아니라 0.035m/s 저속 경로보다 짧은 coordination timeout이었다.

수정 정책은 다음과 같다.

- `WAIT_FRONT_STAGED`에만 90초 제한을 적용하고 다른 substate는 기존 60초 유지
- startup observer는 readiness에 필요한 7개 토픽만 구독
- 전체 DDS state monitor는 기본 비활성화하고 정지 진단에서만
  `AUTO_STATE_MONITOR=true`로 명시적 사용
- 주행 중에는 반복 snapshot 대신 기존 Kiosk/Control Tower UI 사용

## 다음 작업과 운용 제한

전체 `robotctl start`를 반복하지 않는다. 특히 SSH가 불안정해지는 상태에서 세
장비 전체 준비를 다시 시도하지 않는다.

다음 순서는 아래처럼 제한한다.

1. Control Tower의 ID2/ID1 표시는 production pose/visible 토픽을 사용하고 ID0를
   별도 상태로 표시
2. 대기구역 mission yaw는 layout 기준과 허용 오차로 gate하고, PCA yaw가 크게
   벗어나면 PARK를 거절
3. Front 한 대만 10cm 이하 body-X 저속 이동해 CCTV heading/marker offset과
   map/body 변환 중 어느 쪽이 반대인지 확인
4. cold-start 부하와 Rear recovery session 정체를 수정하되 300ms firmware
   watchdog과 host ACK timeout은 완화하지 않음
5. Fleet에 명시적인 fault reset/rearm 계약을 추가하고 UI/Fleet 판단을 일치
6. 다음 자동 시험은 `stop_after_approach=true`로 staging에서 반드시 정지
7. staging 위치를 사람이 확인한 뒤에만 ID0 정렬 단계를 검토

현재는 encoder 기반 예측, 강체 feedback, mission snapshot 업그레이드를 제거할
이유가 없다. 다만 전체 cold-start gate를 통과하기 전에는 이 기능들의 실차 통합
성공을 주장하거나 차량 모션에 사용하지 않는다.
