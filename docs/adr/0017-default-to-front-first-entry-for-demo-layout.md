---
status: accepted
supersedes: 0011-share-simultaneous-entry-between-mission-types.md
---

# 실증 layout은 기존 Front-first entry를 기본으로 사용한다

현재 실증 layout에서는 `simultaneous_entry=true`가 모든 등록 슬롯 P1~P4의 retrieve 접근에서 robot footprint clearance를 위반하므로, 기존에 구현된 sequential Front-first 접근을 실차 기본값으로 사용한다.

park와 retrieve는 동일한 `simultaneous_entry=false` 운용값을 공유한다. Front는 기존 staging 절차를 먼저 수행하고 Rear는 기존 `WAIT_FRONT_STAGED` coordination을 따라 접근한다. 두 로봇의 기존 정렬과 LIFT barrier, relative pose freshness 및 ready/commit protocol은 변경하지 않는다. mission type별 접근 FSM이나 새 planner를 추가하지 않는다.

Fleet preflight도 같은 순서를 모델링한다. 첫 phase에는 Rear가 HOME에 정지한 채 Front route를, 둘째 phase에는 Front가 staging에 정지한 채 Rear route를 기존 nominal speed로 시간 샘플링한다. 각 시점에 oriented robot footprint와 `minimum_inter_robot_gap_m`을 검사한다.

`simultaneous_entry` parameter와 `true` 동작은 삭제하지 않는다. 다른 layout에서는 launch override로 다시 선택할 수 있으며, 동시 경로의 시간 기반 clearance 테스트도 유지한다.
