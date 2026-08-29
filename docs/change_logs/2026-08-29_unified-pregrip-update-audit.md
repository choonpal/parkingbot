# 2026-08-28~29 통합·정렬 후 정지 시험 감사

## 범위와 결론

- 감사 대상: `integration/unified-pregrip-20260829`
- 물리 상태: Front/Rear 전원 OFF, 이번 통합본 미배포, 모션 명령 없음
- 시험 범위: 두 로봇 차량 하부 진입 → 각 차축 정렬 → `aligned_hold`
- 소프트웨어 판정: 아래 수정 후 **PASS**
- 실차 모션 판정: 현장 실측값 확인 전 **NO-GO**

이번 감사는 커밋이 존재하는지만 보지 않고 변경 이유, 현재 구현, 회귀 테스트,
병합 중 최신값 보존 여부를 연결해 확인했다. 처음 상태는 PASS가 아니었으며 실제
회귀 세 가지를 발견해 수정했다.

1. 운영 경로는 640x360을 전달했지만 `cctv_server_dual.launch.py`의 직접 실행
   기본값은 여전히 640x480이었다.
2. 교체 전 640x480 카메라에서 얻은 광축 지상점을 production wrapper가 새
   현장값 위에 강제로 덮어쓰고 있었다.
3. 개발 셸의 `PYTHONPATH`가 기존 `ros2_ws/build`를 먼저 가리켜, 동적 테스트가
   통합 소스 대신 구 설치본을 import할 수 있었다.

수정 후에는 dual-CCTV의 카메라/캘리브레이션 기본값을 모두 640x360으로 맞추고,
현장 launch 값이 repository fallback보다 우선하도록 했다. dual-CCTV 실행에는
두 카메라·BEV·로봇 상태를 보여 주는 읽기 전용 관제탑(기본 5008)도 함께 기동한다.
검증 명령은 통합 소스 디렉터리를 `PYTHONPATH` 첫 항목으로 고정했다.

## 변경 이유와 현재 검증 연결

| 영역 | 당시 문제와 변경 이유 | 현재 구현 | 감사 결과 |
| --- | --- | --- | --- |
| 명령·서보 기동 | 초기 QoS 유실, servo attach/ACK 전 동작 가능성 | reliable command QoS, HELLO→HB→zero probe→servo attach 순서 | 통신/펌웨어 78 tests PASS |
| heartbeat/session | serial open의 DTR reset, 이전 UART 입력, ROS callback 부하가 300 ms watchdog을 건드림 | 0.5 s settle, 입력 drain, session-bound HELLO/ACK, priority UART scheduler, 전용 callback group와 2-thread executor | PASS |
| Front/Rear firmware | 양 로봇 서보 pulse 방향이 반대 | 분리된 Front/Rear ARM profile과 host firmware harness | PASS |
| command owner | APPROACH/ALIGN의 individual node와 DRIVE rigid node가 같은 cmd_vel을 동시에 소유 | phase별 단일 publisher ownership wrapper | PASS |
| ultrasonic | 비활성 phase의 timeout이 전역 hardware fault로 오인됨 | phase-scoped health와 activation/valid-sample gate | PASS |
| perception fail-close | 카메라 단절을 빈 차량/빈 슬롯으로 오인할 수 있음 | camera availability와 observed absence 분리, target/map/empty fail-close | PASS |
| Rear ID0 | 480p 검출·일시 dropout·yaw 튐으로 강체 시험 중단 | 1280x720@8 fps, bounded zero-hold, 3-sample recovery, pose plausibility/median | 진입·정렬 114 tests PASS |
| W/A/S/D 강체 유지 | 이동 중 간격·상대 yaw만 보정하고 전체 진행각이 떠남 | pair heading hold, 연속 gap correction, W/A/S/D 전용 보정 | PASS |
| 정렬 후 정지 | ALIGN 뒤 기존 상태기가 LIFT barrier로 진행 | `stop_after_align`, LIFT ready/commit 차단, `/{role}/aligned_hold` | PASS |
| 운영 기동 | 서로 다른 SHA, 느린 graph 관측, observer 자체 실패가 정상처럼 보임 | 분산 SHA gate, ID0 정적 yaw check, observer prereq/exit 진단 | 운영 38 PASS, 선택 항목 1 skip |
| CCTV/UI | 새 카메라 해상도와 구 기본값 혼재, 구 광학점 강제 적용, 관제탑 별도 실행 | 640x360 일치, 현장 optics 우선, 5008 관제탑 자동 기동, 실제 marker visibility를 drive gate로 사용 | CCTV/UI 94 tests PASS |

## 병합·브랜치 판정

- `af37cf3`의 관제탑·시차 보정은 최신 marker readiness와 merge commit으로 결합했다.
  이 브랜치가 삭제하던 강체/통합 회귀 테스트는 삭제하지 않았고, 일회성 `.bak` 및
  patch 생성 파일만 제외했다.
- `fix/distributed-runtime-preflight`와
  `fix/robot-start-observer-hardening-20260828`은 기존 wrapper/core 구조를 보존한
  파생 커밋으로 이식했다. Rear 카메라 값은 오래된 640x480이 아니라 이후 실측된
  1280x720@8을 유지했다.
- `integration/production-no-keyboard-follow-20260827`은 W/A/S/D 시험 기능을
  제거하므로 이번 목적과 정면 충돌해 병합하지 않았다. 이 브랜치의 ID0 거리
  비활성 조건은 이후 `0.570 m` 실측 기록으로 대체됐다.
- `work/vision-rigid-continuity-20260827`의 측정 geometry, source envelope,
  delayed replay, camera handover guard, 같은-camera relative fallback은 현재의
  `*_production_node` 계열로 구현돼 있다. 하지만 리프트 후 두 상판 마커의 원자적
  카메라 선택과 운반 차량 feedback source provenance는 현재 계열에 없다. 둘 다
  이번 `stop_after_align` 경계 뒤 기능이므로 이번 실차 경로에는 들어오지 않는다.
  전체 인양/운반 제품 통합 완료를 주장하려면 별도 이식·검증이 필요하다.
- retrieval, side-by-side HOME, rear-single-vehicle 브랜치는 각각 다른 mission 또는
  현장 기하 실험이므로 차량 하부 Front-first 정렬 시험에 섞지 않았다.
- site wrapper 추가(`8767e51`)와 즉시 revert(`07465e1`)는 net-zero다. 이후
  calibrated 640x360 전용 `site_jetson.launch.py`가 다시 추가돼 현재 경로가 됐다.

## 현장 자산 대조

2026-08-28 교체 카메라 runtime 자산을 직접 읽었다.

- cam0 intrinsic: 640x360, RMS `0.117275 px`
- cam2 intrinsic: 640x360, RMS `0.140711 px`
- cam0 Homography RMS: `0.017297 m`
- cam2 Homography RMS: `0.007557 m`
- 최종 calibration principal point와 최종 post-swap Homography로 계산한 광축 지상점:
  - cam0 `(2.319423, 2.315810) m`
  - cam2 `(1.891773, 1.296094) m`

자산 SHA256:

```text
0379379ec5651b6c94a30483556b6b36eb0d36e9bd7f6ca5548651b98743b181  cctv0_camera_calibration.npz
3d1ee98385d0d6eaa0441f96b9fd577d4cc28adddf8f5efbe9ae724ee849198e  cctv2_camera_calibration.npz
5127f91578ecbc28235bdf85a90600e33be46e181377aa5c17ddf42cf4dcbf81  homography_cam0_rectified.npy
6d463672972989cac23b45e054764e89b1c2c16f83542fa0fa38fc4a8c1d2941  homography_cam2_rectified.npy
2fde5709d55f8b27ecea2b90d5c7bf2a0434f8070a4dcea1f3da8b78825dcc10  parking_layout.yaml
```

## 재검증 결과

통합 소스를 import하도록 경로를 고정한 결과:

- heartbeat/UART/firmware: `78 passed`
- underbody/pregrip/rigid-pair: `114 passed`
- CCTV/control-tower/UI: `94 passed`
- robotctl/preflight: `38 passed, 1 skipped`
- ROS package 전체: `760 passed, 1 skipped`
- 변경 Python/launch `py_compile`: PASS
- `git diff --check`: PASS

skip은 선택적 로컬 도구 환경 항목이며 ROS·UART·모션·CCTV 계약을 건너뛴 것이
아니다.

## 실차로 넘어가기 전 남은 차단 조건

현재 현장 설정의 아래 네 값은 모두 `0.0`이고, 이것이 실제 장착 측정값이라는
기록을 찾지 못했다.

```text
FRONT_LEFT_SENSOR_X
FRONT_RIGHT_SENSOR_X
REAR_LEFT_SENSOR_X
REAR_RIGHT_SENSOR_X
```

이 값은 `gripper_x - ultrasonic_sensor_x`이며 차축 중심 정지 위치 계산에 직접
사용된다. 네 센서가 실제로 그리퍼 중심과 같은 X선에 있어 `0.0 m`가 맞다는 확인,
또는 실측값 입력 전에는 차량 아래 자동 진입을 실행하지 않는다. wheel radius,
PPR, `lx/ly`, 초음파 부호·threshold·freshness도 readiness 문서의 현장 체크가
완료됐다는 증거를 남겨야 한다.

따라서 현재 결론은 **pregrip 소프트웨어 통합 PASS, 물리 모션 NO-GO**다.
