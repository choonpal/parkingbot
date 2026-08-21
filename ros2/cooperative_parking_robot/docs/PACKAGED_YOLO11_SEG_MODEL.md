# 패키지 포함 YOLO11 차량 Segmentation 모델

## 배포 파일

기본 CCTV launch는 다음 학습 완료 가중치를 사용한다.

```text
share/cooperative_parking_robot/models/parking_vehicle_yolo11n_seg.pt
```

- 원본 아카이브: 개발 장비의 `${HOME}/cam_seg.zip`
- 선택한 원본: `cam_seg/runs/segment/train/weights/best.pt`
- SHA-256: `e60179f0ad4a1b346b1b464dbc0cf93075f1c91385820683b384e238e8c7d896`
- 파일 크기: 6,031,189 bytes
- 기반 모델: `yolo11n-seg.pt`
- checkpoint Ultralytics version: `8.4.123`
- checkpoint license metadata: `AGPL-3.0`
- task: instance segmentation
- class: `0: car`
- 학습 입력 크기: 640
- 학습 epoch: 100

아카이브의 학습 이미지, 라벨, 원본 pretrained 모델과 `last.pt`는 실차 실행에
필요하지 않으므로 저장소에 중복 포함하지 않는다. 기본 launch는
`model_mode:=vehicle_seg`, `inference_imgsz:=640`을 사용하며 외부 `.pt` 또는
TensorRT `.engine`은 기존 `model_path` launch argument로 덮어쓸 수 있다.

## Jetson ROS 2 Humble

이 파일은 PyTorch `.pt` 가중치다. Jetson에서 해당 Ultralytics 버전과 PyTorch가
먼저 설치돼 있어야 하며, 저장소는 인터넷에서 모델을 자동 다운로드하지 않는다.
패키지를 빌드하고 `install/setup.bash`를 source하면 camera launch가 package
share의 모델 경로를 자동으로 사용한다.

사전 점검에서 경로를 명시할 때는 다음과 같이 설치 경로를 사용한다.

```bash
MODEL_PREFIX=$(ros2 pkg prefix cooperative_parking_robot)
hardware_preflight --role jetson \
  --model-path ${MODEL_PREFIX}/share/cooperative_parking_robot/models/parking_vehicle_yolo11n_seg.pt \
  --model-mode vehicle_seg \
  --homography-file $HOME/.ros/adaptive_valet_bot/homography_rectified.npy
```

TensorRT engine은 Jetson의 TensorRT/CUDA/Ultralytics 환경에서 별도로 export하고
실차 검증한 뒤 `model_path`만 변경한다. 이 저장소에는 아직 export된 engine을
포함하지 않는다.

## 검증 한계

제공된 학습 아카이브의 데이터셋은 train 13장, validation 4장으로 작다. 학습
결과가 높더라도 현장 조명, 카메라 높이, 차량 색상과 가림 변화에 대한 일반화를
보장하지 않는다. 실제 배포 전 두 CCTV 각각에서 mask 누락, 오검출, 중심과 yaw,
처리 FPS를 확인하고 필요하면 현장 데이터를 추가해 재학습해야 한다.

이 모델 바이너리는 패키지 소스의 MIT 표기로 재라이선스되지 않는다. 외부 배포
전에는 checkpoint에 기록된 AGPL-3.0 조건, 기반 모델과 학습 데이터의 사용
권한을 프로젝트 책임자가 별도로 확인한다.
