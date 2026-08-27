# 비전 기반 강체 이동: 측정 기하·카메라 handover·지연 보정

## 반영한 실측값

- cam0 optical-axis floor point: `(2.463, 1.982) m`
- cam2 optical-axis floor point: `(1.831, 0.507) m`
- 두 CCTV optical-centre height: `2.610 m`
- Front/Rear 상판 ArUco 높이: `0.120 m`
- 상판 마커는 base_link/회전중심에 있으므로 x offset: `0.0 m`

차량 상판은 기울어져 있고 `0.74 m`는 최고점이므로 단일 수평면 높이로
사용하지 않는다. 알려진 실제 차량 중심과 YOLO raw centre를 비교해 유효
segmentation 높이를 역산하기 전까지 차량 parallax 보정은 0으로 유지한다.

## source-aware 상판 관측

`cctv_robot_marker`는 기존 PoseStamped 외에 역할별
`/{role}/cctv_observation` JSON envelope을 발행한다. 한 메시지 안에 pose,
촬영 timestamp, camera_id, source switch sequence, handover 검증 여부와 seam
bias를 함께 넣으므로 교차토픽 순서에 의존하지 않는다.

cam0을 canonical source로 두고 두 카메라가 같은 마커를 동시에 본 overlap
프레임에서 cam2의 local seam bias를 저역통과 갱신한다. 단순 cost 역전 한 번으로
source를 바꾸지 않고 hold, 최소 개선량과 연속 확인을 통과해야 전환한다.

## PoseFusion rewind/replay

production PoseFusion은 source envelope만 절대 보정 authority로 사용한다.
촬영 시각보다 뒤에 적용된 wheel increments를 1초 이력에서 되감고, 촬영시각
직전 wheel snapshot에서 CCTV correction을 한 번 적용한 뒤 이후 wheel
increments를 다시 재생한다. correction 시각은 최대 한 wheel period로 양자화되며
`/{role}/cctv_fusion_status`에 rewind와 replay 진단을 발행한다.

새 camera source가 overlap에서 검증되지 않았다면 3개의 일관된 프레임을 요구하고,
현재 예측과 12 cm 또는 12 deg 이상 다른 source jump는 거부한다.

## 강체 상대 fallback

ID0가 stale일 때만 쓰는 overhead Front/Rear fallback은 두 source envelope의
촬영시각이 맞고 camera_id가 동일한 경우에만 상대 x/y/yaw correction으로
사용한다. `Front=cam0, Rear=cam2`처럼 mixed-source pair는 폐기한다.

## 검증

- pure Python syntax compile
- source envelope round-trip
- validated/unvalidated source handover
- implausible source jump rejection
- delayed correction rewind + subsequent wheel replay
- measured geometry and intentionally disabled vehicle effective height
