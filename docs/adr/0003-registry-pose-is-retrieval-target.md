---
status: accepted
---

# 이번 실증에서는 Registry pose를 출차 target으로 사용한다

이번 실증에서는 입차 완료 후 출차 요청까지 사람이 차량을 이동시키거나 건드리지 않는다고 보장한다. 따라서 출차 접근 target은 live Perception과 다시 결합하지 않고 Parking Registry에 저장된 보관 차량 자세만 사용한다.

Parking Registry의 차량-슬롯 기록은 최소한 `slot_id`, 최종 차량 자세 `x/y/yaw`, `parking_direction`, vehicle spec, lifecycle state를 포함한다. 이 기록은 활성 미션 reset과 분리되어 현재 Fleet Manager 실행 세션 동안 유지된다. 단순 슬롯 중심보다 차량 배치 완료 시점에 확정한 최종 차량 자세를 우선 저장한다.

Perception/CCTV는 기존의 차량 및 슬롯 관측을 계속 수행하지만, 이번 출차 target 계산에는 detection matching, freshness 검사 또는 live yaw 비교를 사용하지 않는다. 차량 이동 가능성이 있는 실제 운영 환경의 live 검증은 별도 확장으로 남긴다.

`RigidBodySyncNode`는 `ARRIVED`를 발행할 때 기존 `/sync/error_state` JSON에 optional `final_vehicle_pose`와 `plan_stamp_ns`를 포함한다. `final_vehicle_pose`는 `frame_id=map`과 정규화된 `x/y/yaw`를 가진다. Fleet Manager는 현재 활성 미션이 발행한 path 및 `/parking/slot_pose`의 stamp와 `plan_stamp_ns`가 일치할 때만 이를 `pending_final_vehicle_pose`로 받아들인다. 새 topic은 만들지 않는다.

권장 실차 launch에서는 `pose_fusion_node`가 Front/Rear odometry를 `map` frame으로 발행하고 Fleet Manager도 path와 `/parking/slot_pose`를 `map`으로 발행한다. 다만 현재 `RigidBodySyncNode._parse_odom()`에는 odometry frame 검사가 없으므로, 구현 시 `map`이 아닌 odometry를 거부하여 서로 다른 좌표계를 섞지 않도록 한다.

`pending_final_vehicle_pose`는 양쪽 로봇의 `RELEASE_DONE` 뒤에 발행되는 현재 미션의 `RETURN` commit과 일치할 때만 Parking Registry에 복사하고 슬롯을 `OCCUPIED`로 확정한다. 이 차량 배치 완료 경계와 양쪽 로봇의 HOME 복귀 및 전체 미션 완료 경계는 서로 분리한다.
