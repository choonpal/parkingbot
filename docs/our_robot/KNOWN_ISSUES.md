# 확인된 문제 기록

최종 갱신: 2026-08-26

실제 관측되거나 전체 테스트에서 재현된 문제만 기록한다. 추측은 해결 완료로
표시하지 않는다.

## 미해결

### Front bridge의 `STALE_STAMP` 수동 명령 거부

- 심각도: 높음 — 분산 수동 제어의 연속성에 영향 가능
- 재현: 협동 시험 운용 중 robot-2 bridge 로그에서 반복 관측
- 확인 완료: 2026-08-26 현재 robot-1·robot-2 모두 `NTP=yes`,
  `NTPSynchronized=yes`
- 남은 확인: 같은 경고가 새 세션에서도 재현되는지, DDS 지연 명령인지 송신 시각
  문제인지 구분해야 한다. NTP 상태만으로 해결 완료로 보지 않는다.

### robot-1 bridge의 Ctrl+C 종료 예외

- 심각도: 낮음 — 주행 정지에는 영향 없지만 진단 프로세스가 실패 코드로 종료됨
- 재현: 두 실기 진단 모두 수동 제어권 해제와 PWM 0 확인 뒤
  `rcl_shutdown already called on the given context` 예외 발생
- 원인: Humble 기본 SIGINT 처리가 context를 먼저 종료한 뒤 main이
  `rclpy.shutdown()`을 다시 호출한다. robot-2 실험 배포본에는 `rclpy.ok()` 보호가
  있지만 현재 main/robot-1에는 아직 없다.

## 해결됨

### 협동 시험의 robot-1 오도메트리 방향 불일치

- 최초 재현: 3cm 협동 시험 두 번 모두 `명령 반대 방향 이동 감지`로 정지
- 단독 분리 결과: 같은 `+0.0628m/s`, 0.30초 명령에서 robot-2 Front는
  `x=+0.01043m`, robot-1 Rear는 `x=-0.01226m`였다.
- 원인: 명령에는 기체 축 부호를 적용하면서 encoder 정기구학 결과에는 같은
  자기역 부호 변환을 적용하지 않아 robot-1의 ROS X odom이 반전됐다.
- 조치: `EncoderOdometry(axis_sign=...)`를 추가하고 bridge에서
  `axis_sign=self.command_sign`을 전달했다. 실험 브랜치 전체는 병합하지 않았다.
- 검증: 전체 `389 passed, 1 skipped`, 깨끗한 ROS 빌드와 robot-1 원격 빌드 통과.
  배포 후 동일 실기 재시험에서 robot-1 `x=+0.01226m`, robot-2
  `x=+0.01003m`였고, 양쪽 모두 최종 PWM 0과 STM32 ACK를 확인했다.

### 전체 테스트 7개 실패

- 최초 결과: 331 통과, 7 실패
- 원인: 확장된 지도 원점·크기, 통합된 CCTV entry point, 출차 시 지도·fresh odom
  필수조건을 기존 테스트가 반영하지 못함
- 조치: 운영 안전조건은 완화하지 않고 테스트 기대값과 fixture를 현재 코드에
  맞췄으며, 지도 또는 odom이 없으면 출차 요청을 거부하는 회귀 테스트를 추가함
- 1차 재검증: 당시 전체 339개 테스트 통과, 변경 Python 17개 파일 flake8 통과

### 원격 ROS 디렉터리 재업로드 손상

- 최초 결과: 원격 `main` 단독 기준 339 통과, 9 실패, 1 skip
- 원인: 디렉터리 삭제·웹 재업로드 과정에서 패키지 `.gitignore` 누락, Humble
  점검 스크립트 2개의 실행 비트 소실, `__pycache__/*.pyc` 55개 추적, 측정
  wheelbase `0.785m`와 초음파 기본값 테스트 불일치가 함께 발생함
- 조치: `.gitignore`와 실행 비트를 복원하고 pyc 55개를 Git 추적에서 제거했다.
  원격의 측정 wheelbase와 새 카메라 이동 추적 코드는 유지했으며 ArUco 상대 pose
  표시와 함께 3-way 병합했다.
- 최종 재검증: 전체 387개 통과, Node.js가 없는 환경의 내장 JS 문법 테스트 1개
  skip, 변경 Python 19개 파일 flake8 통과, 깨끗한 새 ROS workspace 빌드 통과.
  skip된 항목은 headless Chromium에서 실제 페이지·API·영상 endpoint를 열고 카메라
  카드 2개가 생성되는 것을 확인해 보완했다.

### robot-2 서보 마지막값 불일치

- 최초 상태: 실제 최대 벌림인데 저장값은 `1500/1450us`
- 조치: 이전 파일을 백업하고 robot-2 최대 벌림값 `2600/400us`로 수정
- 재검증: bridge 재기동 시 `(2600, 400)` 복원 요청 확인
