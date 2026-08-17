# Localization 재검증 보고서

> **기록 범위:** 아래 Ubuntu 24.04/OpenCV 4.6 결과는 이전 검증 기록이다. 현재 Humble/Jammy 배포의 실제 rclpy·DDS·카메라 통합 결과가 아니며, `docs/HUMBLE_EXECUTION_ASSESSMENT.md`의 별도 판정을 따른다.

- 검증일: 2026-07-20
- 대상: `cooperative_parking_robot` 실기체 localization 경로
- 기준 문서: `docs/localization_design.md` v1.9
- 판정: **코드 경로 조건부 통과 / 실기체 정확도 검증 보류**

## 1. 결론

정적 코드 검토와 ROS 비의존 회귀 테스트에서 확인된 치명적 문제는 수정했다.
Rear ArUco yaw의 180° 부호 오류, Ubuntu OpenCV 4.6 API 불일치,
CCTV pose/visible 교차토픽 순서 의존, `full_system.launch.py` 배선 누락,
극단적 이상치로의 강제 재획득, calibration 부재 시 잘못된 pose 발행 경로를
보완했다.

그러나 저장소에 실제 `homography_rectified.npy`와 Rear 전용
`rear_camera_calibration.npz`가 없고, 카메라·마커 높이와 부착각도 측정되지
않았다. ROS2 3대 통합 및 실기 주행도 수행하지 못했다. 따라서 현재 상태를
2cm/3° localization 달성이나 현장 배포 가능으로 판정하면 안 된다.

## 2. 수정한 문제

| 문제 | 조치 | 현재 상태 |
|---|---|---|
| Rear ArUco 정면 yaw가 180° | 마커 음의 법선으로 heading 계산, OpenCV 4.6에서 오답인 IPPE 대신 ITERATIVE 사용, offset 추가 | 회귀 테스트 통과 |
| OpenCV 4.6에 `ArucoDetector` 없음 | 신형/legacy API 공통 어댑터 추가 | Ubuntu 24.04/OpenCV 4.6.0 초기화 확인 |
| CCTV pose가 full launch에 없음 | `cctv_robot_marker_node` 추가 | launch 배선 완료 |
| Bool이 pose보다 늦으면 첫 보정 누락 | pose 자체를 현재 프레임 검출 증거로 처리 | 교차토픽 순서 의존 제거 |
| 5회 이상치가 100m 좌표도 강제 수용 | 재획득 위치 0.5m/yaw 45° 상한 추가 | 극단 오탐 반복 테스트 통과 |
| calibration 파일이 없어도 추정값 사용 | 기본 fail-closed, 명시적 opt-in만 예외 | 잘못된 pose의 조용한 발행 차단 |
| 상판 높이 parallax | 나달 카메라 근사 보정 파라미터/로직 추가 | 실측값 0이라 기본 비활성 |
| 카메라 토픽 QoS/촬영시각 손실 | sensor-data QoS, 입력 image stamp 전파 | 코드 반영 |
| wheel/feedback timestamp 안전성 | 역행·중복·stale·future/frame 검사 추가 | 코드 반영 |
| 런타임 의존성 누락 | NumPy/OpenCV/pyserial을 `package.xml`에 선언 | 코드 반영 |

## 3. 수행한 검증

다음 명령과 결과를 확인했다.

```powershell
python -m unittest discover -s src/cooperative_parking_robot/test -p 'test_*.py' -v
```

결과: 6개 모두 통과.

- Rear marker yaw 0°/±20°
- 실제 OpenCV solvePnP 투영/복원 yaw 20°
- 엔코더 카운터 reset 시 모션 폐기
- 반복되는 100m 이상치 차단
- 물리적 상한 안의 일관된 측정 5회째 재획득
- OpenCV 4.6 legacy ArUco detector fallback

```powershell
python -m compileall -q src/cooperative_parking_robot/cooperative_parking_robot src/cooperative_parking_robot/launch
git diff --check
```

결과: Python 컴파일과 diff 형식 검사 통과.

Ubuntu 24.04 WSL의 OpenCV 4.6.0에서도 legacy 경로로
`ArucoDetectorCompat`가 초기화됨을 확인했다. 같은 환경의 합성 투영에서
`SOLVEPNP_ITERATIVE`는 yaw 20°를 복원했다.

## 4. 아직 검증하지 못한 항목

아래 항목은 코드만으로 확정할 수 없으며 현 상태의 배포 차단 조건이다.

1. `/cctv/image_rect` 바닥 기준점으로 만든 실제 `homography_rectified.npy`
2. Rear 카메라의 실제 intrinsic/distortion `rear_camera_calibration.npz`
3. CCTV 광축 바닥 교점, 카메라 높이, Front 상판 마커 높이
4. Front 마커 부착 yaw offset과 Front/Rear 초기 `x/y/yaw`
5. 실제 바퀴 반지름, encoder PPR, `lx/ly`, 최대 정상 tick delta
6. 실제 주행으로 얻은 EKF process/measurement noise
7. Jetson/Front RPi/Rear RPi 간 ROS2 DDS 연결, QoS, clock 동기
8. YOLO와 ArUco 동시 실행 시 프레임레이트·처리/네트워크 지연
9. 장시간 occlusion, 카메라 재획득, STM32 재부팅을 포함한 실패 시험
10. CubeMX `.ioc`와 펌웨어 timer bit-width 매핑 대조

## 5. 남은 설계 위험

- 기본 높이 값 0은 parallax 보정을 끈다. 예시 조건에서는 위치 오차가
  약 9.6cm가 될 수 있어 최종 허용오차 2cm보다 크다.
- `cctv_timeout=0.5s`는 stale 측정을 버리지만 과거 시점으로 되돌려
  재적분하는 지연 보상은 없다. 최대속도 0.08m/s라면 0.5초 동안 4cm를
  이동할 수 있으므로 실측 지연에 따라 허용오차를 넘을 수 있다.
- homography만 사용하는 CCTV 경로는 렌즈 왜곡을 별도로 제거하지 않는다.
- calibration 경로가 상대경로이므로 실제 launch에서는 존재가 확인된
  명시적 절대경로 또는 설치 패키지 share 경로로 지정해야 한다.

## 6. 실기 배포 전 Go/No-Go 체크리스트

- [ ] 두 calibration 파일을 생성하고 노드 시작 시 정상 로드 확인
- [ ] parallax 관련 5개 파라미터와 두 yaw offset을 실측값으로 설정
- [ ] 정적 격자점 전 범위에서 위치 오차 2cm 이하, yaw 오차 3° 이하 확인
- [ ] 직선·제자리 회전·곡선 주행으로 odom과 절대 pose 비교
- [ ] 마커 가림/재등장 시 잘못된 점프 없이 재획득하는지 확인
- [ ] STM32 재부팅과 encoder rollover 시 pose 점프가 없는지 확인
- [ ] 3대 장비 clock skew가 0.1초 이내이고 stale 판정이 정상인지 확인
- [ ] 최악 부하에서 CCTV age와 프레임레이트 기록 후 timeout/noise 재튜닝
- [ ] `localization_status`에서 gate 기각과 forced reacquire를 로그로 확인

위 항목을 통과하기 전 판정은 **No-Go**다. 통과 후 측정 로그와 최종
파라미터를 이 문서에 추가하면 재현 가능한 배포 기준이 된다.
