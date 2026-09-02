# cooperative_parking_robot 문서 안내

현재 운용에서는 저장소 루트의 [현재 통합 상태](../../../docs/CURRENT_INTEGRATION_STATUS.md),
[실차 Runbook](../../../docs/REAL_ROBOT_DEPLOYMENT_RUNBOOK.md),
[실차 준비도](../../../docs/REAL_WORLD_READINESS.md)를 먼저 확인한다.

## 현재 실행·시험 문서

| 문서 | 용도 |
|---|---|
| [차량 하부 진입](UNDERBODY_ENTRY.md) | Front/Rear 진입 기하, phase, `stop_after_align` 검증 |
| [CCTV 파이프라인](RUN_CCTV_PIPELINE.md) | 듀얼 CCTV, BEV, 5008 관제탑 실행·확인 |
| [기능별 테스트](FEATURE_TEST_GUIDE.md) | production node 경계의 소프트웨어 시나리오 |
| [10 cm 협동 직진](COOPERATIVE_DRIVE_TEST.md) | 전체 FSM 없이 두 로봇 방향·정지 확인 |
| [실차 하드웨어 점검](HARDWARE_READINESS.md) | 측정 상수와 단계별 하드웨어 gate |

보정의 현재 실행 절차는 저장소 루트의
[calibration pipeline](../../../docs/pipeline.md)이 기준이다.
`CCTV_CALIBRATION.md`는 intrinsic 원리와 과거 640x480 asset의 이력을 보존한다.

파일명에 날짜나 release 버전이 있는 문서는 당시 설계 기록이다. 현재 기본값과
충돌하면 위 현재 문서와 실제 launch 인자를 우선한다.
