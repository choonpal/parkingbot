---
status: accepted
---

# 안정 Parking Registry 상태를 SQLite에 영속화한다

Fleet Manager는 Parking Registry의 단일 writer를 유지하면서 각 lifecycle
전이를 표준 Python `sqlite3`로 원자적으로 저장한다. 동일한 schema version,
등록 slot 목록과 layout fingerprint가 확인되고 모든 slot이 `EMPTY` 또는
`OCCUPIED`인 경우에만 Fleet 재시작 후 기록을 복원한다. `RESERVED`,
`EXIT_RESERVED`, `EXITING`, 손상 row 또는 layout 불일치는 실제 차량 위치를
추측하지 않고 startup을 중단해 운영자 확인을 요구한다.

차량번호와 final pose/spec/direction은 저장하지만 주차 비밀번호 원문은
저장하지 않고 PBKDF2 iterations, salt, digest만 저장한다. 이 결정은
ADR-0015의 “차량 주차 후 Fleet 재시작 미지원” 제한을 대체한다. 다만 미션
중간 crash 자동 재개와 Perception 기반 물리 상태 reconciliation은 여전히
범위 밖이다.
