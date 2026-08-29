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

현재 원격 Front/Rear Git HEAD는 `ddad9ac`이고 필요한 Python/launch 파일만 overlay
복사됐다. 따라서 controller `a5e7290`과 원격 Git SHA가 같은 clean 배포는 아니다.
`robotctl_core`가 출력한 revision warning을 무시해도 된다는 뜻이 아니며, 다음
정식 실차 gate 전에는 같은 SHA의 clean workspace로 다시 배포해야 한다.

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
