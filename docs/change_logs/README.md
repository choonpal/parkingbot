# Project Change Logs

이 폴더는 프로젝트 코드의 주요 수정 내역을 기록한다.

각 문서는 단순한 Git diff가 아니라 다음 내용을 보존하는 것을 목적으로 한다.

- 왜 수정했는가
- 어떤 문제가 있었는가
- 어떤 코드가 어떻게 바뀌었는가
- 제어/데이터 흐름이 어떻게 달라졌는가
- 어떤 테스트를 했는가
- 실차에서 무엇을 추가 검증해야 하는가
- 어떤 위험요소가 남아 있는가

---

## 2026-08

| 날짜 | 변경 내용 | 주요 영역 |
|---|---|---|
| 2026-08-29 | [통합·정렬 후 정지 시험 감사](./2026-08-29_unified-pregrip-update-audit.md) | Pregrip / CCTV / UART / Operations |
| 2026-08-28 | [기능별 테스트·강체 쌍 실차시험 main 통합](./2026-08-28_feature-tests-and-rigid-pair-field-test.md) | Feature tests / ArUco / Rigid-pair |
| 2026-08-28 | [Production startup heartbeat·freshness·operations 복구](./2026-08-28_production-startup-heartbeat-and-ops.md) | Firmware / ROS Freshness / Operations |
| 2026-08-27 21:27 | [비전 기반 강체 이동: 측정 기하·카메라 handover·지연 보정](./2026-08-27_vision-rigid-handover-and-replay.md) | Vision / Localization / Rigid-body |
| 2026-08-27 20:11 | [Production operation tooling](./2026-08-27_2011_production-operation-tooling.md) | Operations / Deployment / Diagnostics |
| 2026-08-27 19:31 | [Production perception 및 mission gate 보완](./2026-08-27_1931_production-perception-and-mission-gates.md) | Perception / Calibration / Safety |
| 2026-08-27 17:35 | [Mission Safety 및 Runtime 전수 보완](./2026-08-27_1735_mission-safety-and-runtime-fixes.md) | Safety / Perception / Runtime |
| 2026-08-27 (시간 미기록) | [강체제어 P0 Production Guards](./2026-08-27_rigid-body-p0-production-guards.md) | Rigid-body / Safety / Lifecycle |
| 2026-08-27 14:40 | [Production 강체 Mission Reference 및 센서 역할 분리](./2026-08-27_1440_rigid-body-mission-reference.md) | Rigid-body / Sensor Fusion / Safety |
| 2026-08-27 13:58 | [Production 강체 lateral 폐루프 제어](./2026-08-27_1358_rigid-body-lateral-control.md) | Rigid-body / ArUco / Sensor Fusion |
