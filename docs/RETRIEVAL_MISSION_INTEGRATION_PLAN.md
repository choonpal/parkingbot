# 출차 미션 통합 구현 계획

상태: 구현 완료, 차량번호·주차 비밀번호 인증 확장 반영

## 1. 코드 조사 결론

조사 당시 코드에는 주차 슬롯의 차량을 waiting pose로 운반하는 완성된 출차 미션이 없었다. 현재는 이 문서의 integration glue가 구현되어 기존 운반 알고리즘을 park/retrieve 양쪽에서 재사용한다.

기존 `vehicle_entry.exit_longitudinal_translation()`과 `IndividualMoveNode.run_return()`의 `EXIT_UNDERBODY -> EXIT_TO_SIDE -> RETURN_HOME`은 차량을 내려놓은 뒤 로봇이 차량 아래에서 빠져 HOME으로 돌아가는 post-release egress다. 차량 출차 알고리즘은 아니다.

다음 하위 알고리즘과 interface는 park/retrieve 공통으로 그대로 재사용할 수 있다.

- `IndividualMoveNode`의 rear-side staging, `APPROACH -> ALIGN`, wheel scan과 axle 정렬
- Robot FSM의 `LIFT`, `DRIVE`, `RELEASE`, `RETURN` coordination barrier
- `AStarPlanner`의 loaded-footprint 경로 탐색
- `RigidBodySyncNode`의 강체 운반, Pure Pursuit, destination yaw 정렬과 직선 insertion
- `/parking/target_pose`, `/parking/vehicle_spec`, `/virtual_robot/waypoints`, `/parking/slot_pose`
- `/mission/{role}/ready`, `/mission/commit`, `/mission/complete`
- `/ui/mission_request`의 String JSON envelope

새로 필요한 것은 위 알고리즘 사이의 mission orchestration, Registry, 출차 source extraction geometry, 요청/UI 연결과 완료 의미 보정이다.

## 2. 실증 범위와 불변조건

- Fleet Manager 내부 Parking Registry가 슬롯 운영 상태와 차량-슬롯 기록의 단일 writer다.
- 최초 DB 생성은 물리 주차장이 빈 상태일 때만 수행하며 등록 슬롯을 `EMPTY`로 초기화한다.
- 시스템 park가 만든 안정 상태(`EMPTY`, `OCCUPIED`)는 SQLite에 저장해 정상 Fleet 재시작 뒤 복원한다.
- 주차 뒤 사람이 차량을 움직이지 않으므로 retrieve target은 Registry의 final vehicle pose만 사용한다.
- Web UI 재시작은 지원하며 새 `client_id`와 `/fleet/state` snapshot으로 복원한다.
- 미션 중간 상태 자동 재개와 Perception 기반 Registry 재구성은 제외한다. 저장된 transient 상태나 layout 불일치는 startup을 차단한다.
- 출차 중 waiting zone에 새 입차 차량을 두지 않는다.
- 신규 park 운용값은 `parking_direction=forward`다. reverse/unknown 차량의 출차 접근은 제외한다.
- `waiting_polygon`은 vehicle-center detection ROI다. 물리적 parking boundary가 아니다.
- 새 ROS topic, service, action, retrieve 전용 Robot FSM state를 만들지 않는다.

> Stable EMPTY/OCCUPIED Registry records survive Fleet restart; unfinished
> missions require operator recovery.

## 3. 상태 소유권

### Active mission

Fleet가 다음 임시 정보를 소유하고 mission complete 뒤 reset한다.

- `mission_id`, `mission_type`, `request_id`
- `active_source_slot_id`, requested/active destination slot ID, `destination_kind`
- 입차 중 정규화된 차량번호와 salt가 적용된 비밀번호 검증값
- target pose/spec, selected approach와 path publish 여부
- `active_plan_stamp_ns`, `pending_final_vehicle_pose`
- 현재 barrier evidence와 Fleet FSM state

### Parking Registry

Fleet 프로세스 수명 동안 mission reset과 HOME 완료를 넘어 유지한다.

- `slot_id`
- lifecycle
- record를 소유한 reservation/mission ID
- final vehicle pose `x/y/yaw` in `map`
- parking direction
- validated vehicle spec
- 정규화된 차량번호와 salt/PBKDF2 비밀번호 검증값

UI에는 `slot_id`, lifecycle과 Fleet가 계산한 `retrievable`만 제공한다. 차량번호, 비밀번호 검증값, pose, spec과 direction은 노출하지 않는다.

## 4. Parking Registry lifecycle

```text
startup
  -> EMPTY

park PLAN_PATH 성공, path 발행 전
  EMPTY -> RESERVED

matching park RETURN commit
  RESERVED -> OCCUPIED

retrieve 요청 최종 승인
  OCCUPIED -> EXIT_RESERVED

matching retrieve DRIVE commit
  EXIT_RESERVED -> EXITING

matching retrieve RETURN commit
  EXITING -> EMPTY
```

- park 예약 뒤 path 발행 전 오류만 같은 mission의 `RESERVED -> EMPTY` rollback을 허용한다.
- path 발행 뒤 park 실패는 차량 위치가 불명확하므로 `RESERVED`를 유지한다.
- retrieve DRIVE 전 실패는 `EXIT_RESERVED`, DRIVE 뒤 실패는 `EXITING`을 자동 해제하지 않는다.
- `HOME`은 슬롯 lifecycle을 바꾸지 않는다.

## 5. park 전체 흐름

1. Perception이 waiting ROI 차량을 정차 확인하고 기존 target pose/spec/ready를 발행한다.
2. 운영자가 UI에 차량번호, 4~64자 주차 비밀번호와 원하는 빈 `destination_slot_id`를 입력한다. Web은 `POST /api/park`로 받은 값을 기존 `type=park` 요청에 넣어 제출한다.
3. Fleet가 차량번호 형식/중복, 비밀번호 형식, 선택 슬롯의 등록 여부, Registry `EMPTY`와 Perception empty 교집합, freshness, replay, WAIT_TARGET, no-active-mission, robot/fault와 target 조건을 재검사한다. 비밀번호 원문은 검증값을 만든 뒤 보관하지 않는다.
4. Fleet가 request를 ACCEPTED 처리하고 park mission ID/context를 만든 뒤 기존 `WAIT_LIFT`를 즉시 발행한다.
5. 양쪽 FSM이 공통 `APPROACH -> ALIGN -> LIFT`를 수행한다.
6. matching DRIVE commit과 `/robot/lifted=true` 뒤 Fleet가 `PLAN_PATH`로 간다.
7. destination 후보는 Perception empty IDs와 Registry `EMPTY`의 교집합 중 UI가 요청한 슬롯 하나다. legacy park 요청은 기존 자동 선택을 유지한다.
8. slot fit, staging, rotation, insertion corridor와 A*가 모두 성공한 후보를 고른다.
9. path 발행 직전에 current mission과 묶어 `EMPTY -> RESERVED`하고 `active_destination_slot_id`를 저장한다.
10. 동일 stamp로 기존 Path와 `/parking/slot_pose`를 발행한다.
11. RigidBodySync가 ARRIVED를 판정할 때 `map` frame의 실제 vehicle center pose와 `plan_stamp_ns`를 기존 error JSON에 optional field로 넣는다.
12. Fleet는 active plan stamp가 일치할 때만 `pending_final_vehicle_pose`를 보관한다.
13. 양쪽 RELEASE_DONE으로 만든 matching RETURN commit에서 pending pose/spec/forward direction을 저장하고 `RESERVED -> OCCUPIED`로 확정한다.
14. 양쪽 로봇은 기존 post-release egress를 수행하고 각자 return_done 뒤 HOME ready를 발행한다.
15. 양쪽 HOME ready 뒤 Front가 HOME commit을 발행한다. Front는 HOME commit 이후에만 `/mission/complete`를 발행한다.
16. Fleet는 HOME evidence, mission ID와 Registry `OCCUPIED` 확정을 모두 검증하고 `last_completed`를 저장한 뒤 active mission만 reset한다. final pose가 유효하지 않아 Registry 확정이 빠졌다면 성공 완료/reset을 차단한다. OCCUPIED record는 유지한다.

## 6. retrieve 전체 흐름

1. UI는 `/fleet/state.parking_slots`의 세션 상태를 표시하고 운영자에게 차량번호와 주차 비밀번호를 입력받는다. Registry 내부 차량번호나 source slot mapping은 UI로 보내지 않는다.
2. `POST /api/retrieve {vehicle_number, password}`는 request를 제출하고 `submitted`, `request_id`를 즉시 반환한다.
3. Web은 기존 `/ui/mission_request`에 `type=retrieve`, 차량번호와 비밀번호를 발행한다. `source_slot_id`는 보내지 않는다.
4. Fleet가 차량번호/비밀번호를 인증해 source slot을 도출하고 OCCUPIED/forward/final pose/spec, no active mission, target conflict, map/odom/robot/fault freshness를 검증한다. source-slot 단독 요청은 인증 우회 방지를 위해 거부한다.
5. Fleet가 양쪽 HOME odom에서 각 role의 기존 rear-side staging까지 직접 접근 corridor를 사전 검증한다. map boundary, OccupancyGrid, 다른 Registry 차량과 oriented robot footprint를 검사한다.
6. 실증 기본 `simultaneous_entry=false`에서는 Front 이동/Rear HOME 고정, Front staging 고정/Rear 이동의 두 phase를 시간에 따라 샘플링한다. override한 `true`에서는 두 route를 동시에 샘플링한다. 어느 경우든 실제 oriented body clearance가 `minimum_inter_robot_gap_m`보다 작을 때만 거부하며 선분의 단순 교차는 거부 근거가 아니다.
7. 모든 검증 뒤 새 retrieve mission을 만들고 source를 `OCCUPIED -> EXIT_RESERVED`한다.
8. Registry pose/spec을 fresh stamp의 기존 target/spec topic으로 발행한 뒤 Fleet를 `WAIT_LIFT`로 바꾸고 `/fleet/state`를 즉시 발행한다.
9. 양쪽 FSM과 IndividualMove는 park와 동일한 `APPROACH -> ALIGN -> LIFT`를 수행한다. mission type 분기를 추가하지 않는다.
10. matching DRIVE commit에서 `EXIT_RESERVED -> EXITING`한다.
11. Fleet가 선택한 source 차량 영역만 Registry pose/spec으로 OccupancyGrid 복사본에서 제거한다. polygon 없는 COCO/dual 검출의 `car_size_m` 정사각형 raster가 Registry 차량 폭보다 클 수 있으므로, layout이 같은 값을 `source_vehicle_fallback_mask_m`으로 Fleet에 제공한다. source mask는 두 표현 중 큰 외곽과 cell padding만 지우며 원본 Perception map이나 다른 차량 장애물은 변경하지 않는다.
12. retrieve 운반 geometry를 만든다.

   ```text
   stored final vehicle pose
     -> source staging
     -> extraction clear
     -> existing A* from clear
     -> waiting staging
     -> fixed waiting pose via existing final insertion
   ```

13. source staging은 stored pose의 lateral offset을 보존하면서 등록 slot open boundary, loaded half-length와 `slot_staging_gap_m`로 계산한다.
14. extraction clear는 source staging에서 슬롯 바깥 방향으로 `rigid_body_lookahead_m + slot_fit_longitudinal_margin_m` 이상 연장한다. final pose부터 clear까지 동일 slot axis/yaw의 직선 corridor다.
15. source extraction corridor, clear-to-waiting A*, waiting rotation 공간과 waiting staging-to-pose insertion corridor를 기존 oriented loaded-footprint 검사로 확인한다.
16. Path에는 extraction 직선과 clear에서 시작한 A*를 중복 없이 합치고 최종 waypoint는 waiting staging으로 둔다. 기존 `/parking/slot_pose`에는 map-frame waiting pose를 넣어 RigidBodySync의 final insertion을 재사용한다.
17. ARRIVED와 RELEASE 절차는 park와 동일하다. matching retrieve RETURN commit에서만 `EXITING -> EMPTY`하고 source vehicle record를 지운다.
18. 양쪽 return_done -> HOME ready -> HOME commit 뒤 Front가 mission complete를 발행한다.
19. Fleet가 `last_completed`를 저장하고 active mission을 reset한 뒤 다음 요청을 받을 수 있게 한다.

## 7. 기존 topic과 JSON 변경

새 topic은 없다.

### `/ui/mission_request`

기존 topic과 envelope를 유지한다. kiosk park는 차량번호, 비밀번호와 `destination_slot_id`를 추가하고, retrieve는 차량번호와 비밀번호를 보낸다. park의 legacy payload는 하위 호환되지만 retrieve의 `source_slot_id` 단독 요청은 인증 우회 방지를 위해 승인하지 않는다.

```json
{
  "type": "park",
  "vehicle_number": "12가3456",
  "password": "2468",
  "destination_slot_id": "A3",
  "request_id": "ui-...",
  "client_id": "web-...",
  "sequence": 7,
  "stamp_ns": 123456789
}
```

```json
{
  "type": "retrieve",
  "vehicle_number": "12가3456",
  "password": "2468",
  "request_id": "ui-...",
  "client_id": "web-...",
  "sequence": 8,
  "stamp_ns": 123456790
}
```

- 비밀번호는 요청 전달 중에만 존재한다. Web/Fleet 로그, `/fleet/state`, `request_status`, `last_completed`와 Registry record에는 원문을 남기지 않는다.
- 현재 HTTP/ROS transport는 암호화되지 않은 trusted-LAN demo 계약이다.

- client ID가 있으면 `(client_id, sequence)`, 없으면 기존 global sequence로 검사한다.
- non-empty request ID는 별도 bounded recent-ID cache로 중복을 막는다.
- client sequence table과 recent request IDs는 작은 LRU로 제한하고 mission reset과 분리한다.

### `/fleet/state`

기존 필드를 유지하고 다음 optional field를 추가한다.

- `mission_type`
- `active_source_slot_id`
- `active_destination_slot_id`
- `destination_kind`
- `parking_slots: [{slot_id, lifecycle, retrievable}]`
- `request_status: {request_id, type, source_slot_id?, destination_slot_id?, status, reason}`
- `last_completed: {completion_sequence, mission_id, mission_type, source_slot_id?, stamp_ns}`

Registry 변경, 요청 승인/거부, mission start/reset과 completion 시 timer를 기다리지 않고 한 번 즉시 발행한다. 기존 `empty_count`는 유지한다.

### `/sync/error_state`

기존 `error`와 진단 필드는 유지한다. `error=ARRIVED`일 때만 다음 optional field를 추가한다.

```json
{
  error: ARRIVED,
  plan_stamp_ns: 123456789,
  final_vehicle_pose: {
    frame_id: map,
    x: 1.2,
    y: 3.4,
    yaw: 1.5708
  }
}
```

Robot FSM은 기존처럼 `error`만 읽으므로 호환된다. Fleet만 plan stamp와 map frame을 추가 검증한다.

### coordination topics

`/mission/{role}/ready`와 `/mission/commit`의 기존 JSON 형식은 그대로 두고 허용 `stage`에 `HOME`을 추가한다. `/mission/complete` 형식은 변경하지 않는다.

## 8. 요청 결과와 UI

- HTTP 응답: `accepted` 대신 `submitted`와 request ID를 반환한다.
- 실제 승인/거부는 Fleet만 결정하고 `request_status`에 기록한다.
- 안정적인 reason code를 사용한다: `INVALID_REQUEST`, `INVALID_VEHICLE_NUMBER`, `INVALID_PASSWORD`, `VEHICLE_ALREADY_PARKED`, `VEHICLE_OR_PASSWORD_INVALID`, `DESTINATION_SLOT_NOT_FOUND`, `DESTINATION_SLOT_NOT_EMPTY`, `DESTINATION_SLOT_UNAVAILABLE`, `DUPLICATE_REQUEST_ID`, `DUPLICATE_SEQUENCE`, `STALE_REQUEST`, `MISSION_ALREADY_ACTIVE`, `UNSUPPORTED_PARKING_DIRECTION`, `MISSING_VEHICLE_RECORD`, `ROBOT_NOT_IDLE`, `APPROACH_CORRIDOR_BLOCKED`.
- UI는 reason code를 한국어 표시 문자열로 변환한다.
- UI는 차량번호 mapping, 비밀번호 검증값, pose/spec/direction을 받지 않으며 slot lifecycle과 `retrievable`만 렌더링한다.
- browser는 자신이 제출한 request ID와 `request_status.request_id`가 같을 때 결과를 표시한다.
- `completion_sequence`가 직전 처리값보다 커질 때만 완료 toast를 한 번 띄운다. 첫 status poll은 현재 sequence를 기준값으로 잡아 Web 재시작 때 과거 완료를 다시 알리지 않는다.

## 9. 파일별 변경 계획

### 신규 코드

- `cooperative_parking_robot/parking_registry.py`
  - Fleet가 단일 writer로 사용하는 Registry module
  - lifecycle과 mission binding 검증
  - park reservation/placement, retrieve reservation/transport/release 전이
  - 차량번호 정규화/중복 방지와 salt/PBKDF2 비밀번호 검증
  - UI-safe summary와 `retrievable` 계산
  - SQLite schema/layout 검증과 안정 상태 startup 복구
  - transient/corrupt 상태 fail-closed, ROS publisher 없음

### 핵심 실행 코드

- `cooperative_parking_robot/fleet_manager_node.py`
  - Registry와 active mission metadata 소유
  - park 차량번호/credential과 requested destination slot을 reservation에 결합
  - retrieve 차량번호/비밀번호 인증 후 source slot 자체 결정
  - park/retrieve 요청 분기, client/request replay guard와 request status
  - existing target/spec publisher를 retrieve 때만 추가 사용
  - robot state/fault, target ready, sync status, ready/commit 구독
  - park candidate를 Perception-empty와 Registry-empty의 교집합으로 제한
  - park 예약 경계와 rollback, commit 기반 lifecycle 반영
  - retrieve 접근 사전검사와 transport path 조립
  - source-car-only planning-grid mask
  - fixed waiting pose, plan stamp/final pose correlation
  - parking slot summary, last completed와 즉시 Fleet state 발행
  - mission reset에서 Registry/replay/completion history를 보존

- `cooperative_parking_robot/parking_geometry.py`
  - stored final pose에서 slot-axis source staging/clear 계산
  - fixed waiting pose의 staging 계산
  - path point deduplication과 oriented corridor에 필요한 순수 geometry
  - 기존 park candidate와 direction 계산은 유지

- `cooperative_parking_robot/vehicle_entry.py`
  - 기존 role별 rear staging 수식을 재사용하는 retrieve preflight 함수
  - direct route의 시간 샘플과 동시 robot clearance 검사
  - IndividualMove runtime 접근 알고리즘은 변경하지 않음

- `cooperative_parking_robot/freshness.py`
  - bounded `client_id -> last_sequence`와 recent request-ID replay guard
  - legacy global sequence 경로 유지

- `cooperative_parking_robot/rigid_body_sync_node.py`
  - odom `frame_id=map` 검증
  - ARRIVED 당시 control-point vehicle center pose/yaw normalize
  - existing error JSON에 optional `plan_stamp_ns/final_vehicle_pose`
  - Pure Pursuit, lookahead, final control은 변경하지 않음

- `cooperative_parking_robot/robot_state_machine_node.py`
  - coordination stage validator에 HOME 추가
  - RETURN에서 return_done 뒤 HOME ready/commit 대기
  - HOME commit 뒤에만 Front complete 발행 및 양쪽 reset/IDLE
  - APPROACH/ALIGN/LIFT/DRIVE/RELEASE와 simultaneous-entry 로직은 변경하지 않음

- `cooperative_parking_robot/jetson_vision_web_node.py`
  - process UUID `client_id`, 공통 request builder
  - 차량번호/비밀번호를 받는 `POST /api/park`, `POST /api/retrieve`와 `submitted` HTTP 의미
  - 입차용 동적 EMPTY slot 선택과 Registry slot 상태 표시
  - 민감값을 제외한 mission request 로그
  - request status reason 표시와 completion-sequence 단발 toast
  - Fleet snapshot만 UI 운영 상태의 source로 사용
  - `enable_operator_ui=true` 기본 운용에서는 raw CCTV kiosk/API를 실행하고,
    `enable_debug_overlay=false`이면 YOLO/ArUco/FPS overlay와 annotated topic을 끔

- `launch/cctv_server*.launch.py`, `launch/full_system.launch.py`
  - operator UI 실행 조건을 debug overlay flag와 분리
  - Fleet가 UI 승인을 요구하는 single/dual 실차 기본값에서는 kiosk가 자동 실행됨

### 설정/등록

- `cooperative_parking_robot/bev_layout_core.py`
  - waiting polygon 중심을 Fleet `waiting_x/y`로 기록
  - 명시적으로 전달된 `waiting_yaw_deg` 기록
  - Fleet에도 center ROI 검사용 waiting polygon 기록
  - 생성 parking direction을 forward로 변경
  - waiting polygon 주석을 vehicle-center detection ROI로 유지

- `cooperative_parking_robot/bev_layout_calibrator_node.py`
  - 임의 추정 없이 명시적 waiting yaw 설정을 layout writer에 전달하고 preview/status에 표시

- `launch/bev_layout_calibration.launch.py`
  - `waiting_yaw_deg` launch argument 추가

- `config/parking_layout.yaml`
  - `waiting_yaw_deg`와 Fleet용 waiting polygon 추가
  - `parking_direction: forward`
  - Fleet preflight가 IndividualMove와 공유해야 하는 nominal entry geometry/speed 값 명시
  - Fleet의 `rigid_body_lookahead_m=0.15`가 `sync_params.yaml`의 lookahead와 같음을 주석 및 테스트로 고정

### 변경하지 않는 실행 알고리즘

- `individual_move_node.py`
- `pure_pursuit.py`
- `astar_planner.py`
- `cctv_merge_node.py`
- `yolo_bev_map_node.py`
- `simultaneous_entry` parameter와 true/false 기존 실행 로직. 실차 launch/layout 기본 운용값만 `false`로 변경한다.

필요한 launch/config 전달과 문서 주석만 조정하며 위 알고리즘 내부에는 park/retrieve 분기를 넣지 않는다.

### 문서

- `CONTEXT.md`와 `docs/adr/0001...0017`
- `docs/pipeline.md`
- package `README.md`, `docs/MASTER_PLAN.md`

현재 “출차 미구현” 설명을 실제 연결 방법, demo invariant와 안정 상태 Fleet restart 복구 범위로 갱신한다.

## 10. TDD seam과 vertical slices

테스트 seam은 다음 공개 interface로 한정한다.

1. `ParkingRegistry`의 lifecycle/query interface
2. `parking_geometry`와 `vehicle_entry`의 순수 geometry result
3. 기존 ROS JSON topic과 Flask HTTP interface
4. 기존 Robot FSM ready/commit/complete protocol

내부 helper 호출 횟수나 private field는 검사하지 않는다. 각 slice는 failing test 하나를 먼저 만들고 최소 구현으로 통과시킨 뒤 다음 slice로 이동한다.

### Slice 1 — Registry

- startup은 등록 슬롯 전부 EMPTY
- 잘못된 lifecycle 또는 mission ID 전이 거부
- park `EMPTY -> RESERVED -> OCCUPIED`
- retrieve `OCCUPIED -> EXIT_RESERVED -> EXITING -> EMPTY`
- active mission reset/HOME 뒤 OCCUPIED record 유지
- 새 DB는 전부 EMPTY, 같은 DB의 EMPTY/OCCUPIED는 Registry 재생성 뒤 복원
- schema/layout/slot 불일치와 transient lifecycle startup 차단
- summary에 내부 pose/spec/direction이 노출되지 않음
- 차량번호 정규화/중복 거부, 서로 다른 salt와 비밀번호 검증
- summary에 차량번호·credential이 노출되지 않음

### Slice 2 — UI request replay

- 같은 client ID의 증가 sequence만 허용
- 새 client ID는 sequence 1 허용
- legacy no-client global sequence 유지
- duplicate request ID가 park/retrieve 모두 차단
- LRU bound와 mission reset 독립성
- stale/future envelope 거부

### Slice 3 — HOME barrier

- 한쪽 return_done만으로 complete가 나오지 않음
- 양쪽 HOME ready 전 HOME commit이 나오지 않음
- 양쪽 ready 뒤 HOME commit 한 번
- Front complete는 HOME commit 뒤 한 번
- Rear도 HOME commit 뒤 IDLE
- stale/wrong mission HOME/RETURN 무시

### Slice 4 — ARRIVED pose

- known map-frame odom에서 계산한 vehicle center x/y/yaw literal 검증
- yaw normalization
- non-map odom 거부
- optional field를 모르는 Robot FSM이 기존 error만 읽음
- wrong/stale plan stamp를 Fleet가 pending pose로 받지 않음

### Slice 5 — park reservation

- Perception empty와 Registry EMPTY 교집합만 후보
- slot fit/corridor/A* 실패 시 예약 없음
- 성공 시 path publish 전 RESERVED
- pre-publish failure만 rollback
- post-publish failure는 RESERVED 유지
- matching park RETURN만 OCCUPIED와 record 저장

### Slice 6 — retrieve request/entry

- 차량번호/비밀번호 불일치와 source-slot-only 우회 요청 거부
- unsupported direction/missing record 거부
- active mission/target conflict/stale robot/map/odom/fault 거부
- 승인 시 EXIT_RESERVED, fresh target/spec, immediate WAIT_LIFT
- IndividualMove가 기존 target interface로 같은 staging을 생성

### Slice 7 — approach preflight

- wall/map boundary/other parked vehicle 충돌 거부
- oriented robot footprint가 통과하는 direct route 허용
- 선분은 교차하지만 서로 다른 시각에 통과하는 두 route 허용
- 같은 시각 body gap 위반 route 거부
- 현재 예시 HOME/slot geometry의 P1~P4가 simultaneous mode에서 모두 거부됨
- 같은 P1~P4가 기존 sequential Front-first mode에서는 모두 허용됨

### Slice 8 — retrieve transport path

- source staging이 open boundary + loaded half-length + gap을 만족
- clear extension이 lookahead + margin 이상
- lookahead만큼 clear 전을 보더라도 loaded footprint가 slot 밖
- final->staging->clear 동일 축/yaw
- selected source 차량 grid cell만 mask되고 다른 차량은 남음
- blocked extraction/waiting insertion/map boundary 거부
- A* first point dedup 및 waypoint continuity
- Path final point는 waiting staging, slot pose는 fixed waiting pose
- waiting center가 ROI 내부인지 검증하되 full-footprint ROI containment를 요구하지 않음

### Slice 9 — UI와 completion

- HTTP는 `submitted`이고 Fleet ACCEPTED/REJECTED와 구분됨
- slot list는 Registry summary만 사용
- park는 차량번호/비밀번호/선택 EMPTY slot, retrieve는 차량번호/비밀번호만 제출
- 차량번호/비밀번호/pose/spec/direction이 status/API response와 로그에 없음
- request ID가 맞는 결과만 표시
- completion sequence 증가 때 toast 한 번
- Web restart 첫 poll은 state를 복원하지만 과거 completion toast를 반복하지 않음

### Slice 10 — park -> retrieve session

- park 완료는 RETURN에서 OCCUPIED, HOME 뒤 last_completed/reset
- UI가 OCCUPIED/retrievable slot을 복원
- retrieve 완료는 RETURN에서 EMPTY, HOME 뒤 last_completed/reset
- 다음 park 요청 수락 가능
- 다른 mission ID의 ARRIVED/commit/complete가 state나 Registry를 바꾸지 않음

## 11. 기존 park regression

- target만으로 UI-confirmed park가 자동 시작되지 않음
- park 승인 timeout과 one-shot 소비
- 기존 WAIT_LIFT/APPROACH/ALIGN/LIFT/DRIVE/RELEASE/RETURN 순서
- simultaneous-entry true의 peer staged/PREALIGNED barrier와 false fallback
- vehicle spec wheelbase 전파와 safety validation
- loaded-footprint slot fit, A*, rotation space와 insertion corridor
- Path와 slot pose 동일 stamp 결합
- Rigid Pure Pursuit/final insertion 동작
- post-release robot egress 순서
- mission reset 뒤 두 번째 park 가능
- optional Fleet/error JSON field를 기존 consumer가 무시
- single/dual CCTV launch에서 mission output publisher ownership 유지
- waiting ROI의 기존 center-in-polygon 감지와 0.4 x 0.6 m 예시 동작 유지

기능 구현 전 기준선은 ROS 2 Jazzy에서 전체 테스트 162개 통과였다. 출차 통합 뒤 기준선은 203개였으며 차량번호/비밀번호 확장과 최종 보강 뒤 전체 결과는 218개 통과다. 실차 배포 대상인 ROS 2 Humble 호환성은 표준 Python 3.10 API와 기존 ROS message/QoS 계약을 유지하고 현장 regression으로 확인한다.

## 12. 완료 조건

- 새 topic/service/action 없이 실제 UI에서 선택 슬롯 park와 차량번호/비밀번호 인증 retrieve가 시작된다.
- park/retrieve 모두 양쪽 HOME commit 전에는 mission complete와 다음 요청 수락이 발생하지 않는다.
- park가 만든 OCCUPIED record가 mission reset 뒤 유지되고 retrieve source가 된다.
- retrieve source는 RELEASE가 확인된 RETURN commit 전에는 EMPTY가 되지 않는다.
- 기존 park 동작과 existing entry/transport/egress 알고리즘이 regression suite를 통과한다.
- demo restart 제한과 no-human-movement/no-new-waiting-car 전제가 운용 문서에 명시된다.
