---
status: superseded
superseded_by: 0017-default-to-front-first-entry-for-demo-layout.md
---

# park와 retrieve가 같은 simultaneous entry parameter를 사용한다

park와 retrieve가 기존 `simultaneous_entry` parameter를 공유하고 mission type별 접근 분기를 만들지 않는 결정은 유지한다. 실차 기본값 `true` 결정만 P1~P4 시간 기반 clearance 검증 결과에 따라 ADR 0017로 대체되었다.

동시 진입은 두 로봇이 같은 위치를 향한다는 뜻이 아니다. Front와 Rear는 기존 vehicle-frame geometry가 정한 서로 다른 rear-side staging 위치로 동시에 이동하고, 양쪽 staging 완료를 `WAIT_PEER_STAGED` barrier에서 확인한 뒤 `PRE_ALIGN -> SCAN_IN`으로 진행한다. 기존 relative-pose freshness, peer staging 및 `PREALIGNED` 검사를 그대로 적용한다.

Front는 coordination commit을 조정하지만 retrieve에서 물리적 접근을 단독으로 먼저 시작하지 않는다. `IndividualMoveNode`의 `APPROACH`, `ALIGN`, `SCAN_IN`에 mission type 분기를 추가하지 않는다. `simultaneous_entry=false`의 기존 Front-first 동작은 parameter fallback 및 운영 선택 기능으로 보존한다.
