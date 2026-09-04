# 카메라·Calibration·실측값 기준

갱신: 2026-09-04. 카메라 구성은 최종 검토자료와 운영자의 팀 보유 확인을 반영했다.
부품 수량·재고·비용은 [BOM](BOM.md), 배선은 [전장 제작서](ELECTRICAL_WIRING.md),
보정 작업 순서는 [pipeline](../pipeline.md)이 담당한다.
이 문서는 역할과 값의 기준 위치를 연결하며 calibration 파일의 수치를 복제하지 않는다.

## 카메라 역할

| 위치 | 실물 | 담당 장비 | 현재 역할 |
|---|---|---|---|
| 천장 2대 | 팀 보유 Logitech C920, C922 각 1대 | Jetson | 차량 segmentation, 상판 ID1/ID2, 공통 map/BEV |
| Rear 전방 | OV2710 | robot-1 | Front 후면 ID0 상대 x/y/yaw |
| Front | OV2710 | robot-2 | 보조 영상·확장용; 현재 Front production launch에는 필수 카메라 노드 없음 |

`cam0`와 `cam2`는 논리 라벨이다. C920/C922 중 어느 장치가 어느 라벨인지는
현장 `/dev/v4l/by-path/`와 실제 영상으로 확인한다. 모델명만으로 배정하지 않는다.

## Calibration 자산

| 역할 | 기준 파일/위치 | 적용 조건 |
|---|---|---|
| Jetson cam0 intrinsic | `~/.ros/adaptive_valet_bot/cctv0_camera_calibration.npz` | 해당 물리 카메라·초점·운용 해상도와 일치 |
| Jetson cam2 intrinsic | `~/.ros/adaptive_valet_bot/cctv2_camera_calibration.npz` | cam0와 별개 파일 |
| cam0 rectified Homography | `~/.ros/adaptive_valet_bot/homography_cam0_rectified.npy` | 보정된 영상과 동일 map 원점·단위 |
| cam2 rectified Homography | `~/.ros/adaptive_valet_bot/homography_cam2_rectified.npy` | 두 카메라 overlap 정합 확인 |
| 주차장 layout | `~/.ros/adaptive_valet_bot/parking_layout.yaml` | slot·corridor·waiting·no-go 실측 |
| Rear ID0 intrinsic | Rear의 `~/.ros/adaptive_valet_bot/rear_camera_calibration.npz` | 실제 launch의 calibration 경로를 우선 대조 |

Jetson 현장 파일이 실제 배포 기준이며, 저장소의 `config/*.npz`와 같다고 가정하지 않는다.
현재 CCTV launch 기본 해상도는 640×360, Rear ID0 launch는 1280×720이다.
해상도나 물리 카메라·초점을 변경하면 해당 intrinsic과 Homography를 재검증한다.

## 마커·기구 실측값의 기준 위치

| 항목 | 현재 코드/문서 기준 | 해석 |
|---|---|---|
| 천장 상판 마커 | [pipeline](../pipeline.md), [CCTV launch](../../ros2/cooperative_parking_robot/launch/cctv_server_dual.launch.py) | DICT_4X4_50, Rear=ID1, Front=ID2, 검은 정사각형 한 변 0.24m |
| Front 후면 ID0 | [Rear launch](../../ros2/cooperative_parking_robot/launch/rear_robot.launch.py) | Rear 관측, 검은 정사각형 한 변 0.10m; 부착판 크기 아님 |
| ID0 정렬 offset | [id0_calibration.yaml](../../ros2/cooperative_parking_robot/config/id0_calibration.yaml) | 거리·횡방향·yaw 장착 보정의 단일 설정 파일 |
| 카메라 광축 지상점·높이와 검출 평면 | [site_geometry.py](../../ros2/cooperative_parking_robot/cooperative_parking_robot/site_geometry.py)와 현장 launch 인자 | 광축 지상점과 카메라 케이스 수직투영점을 구분; 현장 인자 우선 |
| 차량 wheelbase·로봇 외곽·강체 설정 | [sync_params.yaml](../../ros2/cooperative_parking_robot/config/sync_params.yaml), Front/Rear launch | 설정값은 현장 실물과 대조; 차량 질량을 뜻하지 않음 |
| 초음파-그리퍼 X offset | Front/Rear launch, [Runbook](../REAL_ROBOT_DEPLOYMENT_RUNBOOK.md) | `gripper_x - ultrasonic_sensor_x`; 네 기본값 0.0m, 재장착 후 재측정 |
| 휠·엔코더·핀·전압 | [전장 제작서](ELECTRICAL_WIRING.md), [시험 기록](TEST_LOG.md) | 5,182는 기준 모터의 출력축 4체배 카운트; 모든 모터의 보증값 아님 |

상판 예비 ID3/ID4는 현재 운용 ID가 아니다. Rear 단독 실험의 ID2/ID3 설정과도 섞지 않는다.
원본 intrinsic/Homography 수치, marker 크기와 장착 offset은 서로 다른 보정 항목이다.

## 변경 시 함께 기록할 내용

장치 역할, 촬영 해상도·초점, 보정일, 자산 SHA-256, Homography RMS·공통 기준점,
마커 한 변·부착 방향·높이, 실제 launch override와 검증 영상 위치를 남긴다.
공개 문서에는 로그인 정보·내부 IP·장치 일련번호를 적지 않는다.

역할 분담의 소프트웨어 설명은 [시스템 인수인계](SYSTEM_HANDOFF.md),
시연 장면과 원본 정보는 [검증 기록](../FINAL_VALIDATION_2026-09-04.md)이 기준이다.
