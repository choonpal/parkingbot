# ROS 2 Humble 실행 가능성 검토 결과

> **과거 기록 — 날짜·버전 미기재 초기 Humble 이식 검토 스냅샷.** 현재 실행 절차와
> 검증 판정은 저장소의 `docs/REAL_ROBOT_DEPLOYMENT_RUNBOOK.md`와
> `docs/REAL_WORLD_READINESS.md`를 따른다.

## 판정

**조건부 실행 가능.**

- ROS 2 Humble 소스 패키지·launch 구조: 실행을 목표로 정리됨
- 하드웨어 없는 한-PC smoke launch: Humble 환경에서 실행 가능하도록 구성됨
- Jetson + Front RPi + Rear RPi 분산 launch: 외부 파일·카메라·UART가 준비되어야 함
- ZIP만으로 실제 차량 인양·운반: 실행 불가

## Humble 이식에서 확인한 사항

- 현재 Python 문법은 Python 3.10에서 사용할 수 있는 범위다.
- `rclpy` 기본 Node, publisher/subscription/timer, sensor-data QoS를 사용한다.
- 임무 Path와 최종 슬롯은 reliable + transient-local QoS를 사용한다.
- launch는 `LaunchDescription`, `DeclareLaunchArgument`, `IfCondition`, `LaunchConfiguration`, `Node`, `ParameterValue`, `FindPackageShare` 범위다.
- 카메라 입력은 노드 파라미터와 launch 인자로 변경했다.
- Humble 환경·ABI·장치 검사를 `hardware_preflight`에 추가했다.

## 현재 ZIP만으로 막히는 항목

1. YOLO/TensorRT 모델 없음
2. homography 없음
3. 천장 calibration은 포함됐으나 Rectified Homography와 Rear 카메라 calibration은 없음
4. 카메라 publisher 없음
5. 완전한 STM32CubeMX/HAL 프로젝트 없음
6. 실측 구동·기구 상수 없음
7. 실제 ROS 2 Humble 세 장비 통합시험 미수행

## 통과 기준

다음 순서가 모두 성공해야 “실행 가능”을 넘어 “실차 시연 가능”으로 판정한다.

```text
Humble preflight PASS
→ colcon build/test PASS
→ smoke launch 10분 무오류
→ 카메라 토픽/YOLO/homography PASS
→ 각 RPi UART/GPIO/odom PASS
→ 세 장비 DDS PASS
→ 바퀴 공중시험 PASS
→ 빈 차체 협조주행 PASS
→ 저하중 인양·운반 PASS
→ ESTOP/통신단절 PASS
```

현재 자동 검증은 소스·단위테스트·패키징·STM32 C 문법 범위이며, 실기체 결과를 대신하지 않는다.

## 이번 수정본에서 수행한 자동 검증

| 항목 | 결과 |
|---|---:|
| Python `compileall` | PASS |
| Python 3.10 문법 모드 AST 파싱 | PASS, 30 files |
| pytest | PASS, 38 tests |
| `setup.py check` | PASS |
| Python wheel 생성·압축 무결성 | PASS |
| package.xml XML 파싱 | PASS |
| config YAML 파싱 | PASS |
| launch description stub 구성 | PASS, 4 launch files |
| shell script `bash -n` | PASS |
| STM32 C GNU11 `-Wall -Wextra -Werror -fsyntax-only` | PASS |

검증 호스트는 ROS 2 Humble이 설치되지 않은 Debian 13/Python 3.13 환경이었다. 따라서 실제 `colcon build`, launch 프로세스 생성, DDS 통신, `cv_bridge`, TensorRT, GPIO, UART 동작은 이 환경에서 실행하지 못했다. 이를 숨기지 않고 **Humble 실기 통합은 미검증**으로 판정한다.
