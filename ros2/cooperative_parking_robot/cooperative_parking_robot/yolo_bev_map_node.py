#!/usr/bin/env python3
"""
==================================================
[1-1] yolo_bev_map_node
==================================================
천장 카메라 영상 → 맵 생성 + 빈자리/타겟 포착

입력:
  /cctv/image_rect (sensor_msgs/Image) — 왜곡 보정된 천장 카메라
출력:
  /parking/map (nav_msgs/OccupancyGrid) — 주차장 2D 지도
  /parking/target_pose (geometry_msgs/PoseStamped) — 타겟 차량 좌표
  /parking/empty_slots (geometry_msgs/PoseArray) — 빈자리 좌표들

YOLO 모델 + 호모그래피(BEV). 90도 수직 천장 카메라.

모델 통합 (v1.9):
  YOLO11n-seg 한 번으로 차량 mask를 얻고, 고정 등록된 주차면과의 겹침률로
  점유 여부를 계산한다. 따라서 empty_slot 클래스를 따로 학습할 필요가 없다.
  차종 분류(EfficientNetV2-B0)는 별도 유지 — 차량 최초 진입 시
  crop 이미지 1회 분류 → 제원(휠베이스) 매핑 → /parking/vehicle_spec

듀얼 카메라 sensor 모드 (v1.11, docs/DUAL_CCTV_MERGE_20260812.md):
  천장 카메라를 2대 이상 쓰면 이 노드를 카메라마다 하나씩 띄운다. 이때
  ``publish_mission_outputs:=false``로 두어 /parking/* 임무 토픽을 직접
  발행하지 않게 하고, ``publish_detections:=true``로 자기 카메라가 본
  차량 목록만 ``<detection_topic>``(std_msgs/String JSON)에 실어 보낸다.
  최종 /parking/* 판단은 cctv_merge_node가 두 카메라를 합쳐서 내린다.

  파라미터 기본값은 종전과 같은 단일 카메라 독립 동작(mission 토픽 직접
  발행, detection 미발행)이므로 기존 cctv_server.launch.py는 그대로다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import Bool, String

from cooperative_parking_robot.vision_utils import (
    correct_floor_projection,
    directed_axis_yaw,
    load_yolo_model,
    normalize_model_mode,
    parse_class_ids,
    principal_axis_yaw,
)
from cooperative_parking_robot.latest_qos import SENSOR_LATEST_QOS
from cooperative_parking_robot.parking_geometry import (
    parse_registered_slots,
    polygon_overlap_ratio,
    slot_polygon,
)
from cooperative_parking_robot.bev_fusion_core import (
    CameraDetection,
    encode_detection_envelope,
    image_corner_coverage,
)
import math
import os
import time

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    from ultralytics import YOLO
    DEPS_OK = True
except ImportError:
    DEPS_OK = False


class YoloBevMapNode(Node):
    def __init__(self):
        super().__init__('yolo_bev_map_node')

        # ===== 파라미터 =====
        self.declare_parameter('image_topic', '/cctv/image_rect')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('model_mode', 'coco')
        self.declare_parameter('cls_vehicle', 0)      # 커스텀 클래스: 차량
        self.declare_parameter('cls_empty_slot', 1)   # 커스텀 클래스: 빈자리
        # vehicle_seg 모드는 empty_slot을 학습하지 않고 고정 슬롯 DB와
        # 차량 마스크의 겹침률로 빈자리를 판정한다.
        self.declare_parameter('use_fixed_slots', True)
        self.declare_parameter('slot_occupancy_overlap_ratio', 0.10)
        self.declare_parameter('coco_vehicle_class_ids', [2, 3, 5, 7])
        self.declare_parameter('inference_imgsz', 320)
        self.declare_parameter('process_every_n', 3)
        self.declare_parameter('require_dependencies', False)
        self.declare_parameter('require_homography', False)
        self.declare_parameter('layout_registered', False)
        self.declare_parameter('require_registered_layout', False)
        self.declare_parameter('classifier_path', 'efficientnetv2_b0_vehicle.pt')
        # YOLO 인식/맵은 유지하되, 한 종류 모형차 실증에서는 휠베이스를 고정한다.
        self.declare_parameter('use_fixed_wheelbase', True)
        self.declare_parameter('fixed_wheelbase_m', 0.785)
        self.declare_parameter('homography_file', 'homography_rectified.npy')
        # 브라우저 등록 도구가 저장한 H는 픽셀->metre를 직접 출력한다.
        self.declare_parameter('homography_scale_to_m', 1.0)
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width_m', 4.40)
        self.declare_parameter('map_height_m', 3.83)
        self.declare_parameter('confidence', 0.4)
        # Vehicle-axis estimate from a segmentation-mask PCA. A plain COCO
        # detection model has no mask and therefore retains the safe yaw=0
        # fallback until a parking segmentation model is supplied.
        self.declare_parameter('yaw_pca_min_ratio', 1.25)
        self.declare_parameter('yaw_ema_alpha', 0.15)
        self.declare_parameter('waiting_yaw_deg', 0.0)
        self.declare_parameter('yaw_limit_deg', 90.0)
        # Until oriented obstacle boxes are available, use the longer vehicle
        # dimension as a conservative square rather than the old 0.20m point.
        self.declare_parameter('car_size_m', 0.90)
        self.declare_parameter('stationary_tolerance_m', 0.02)
        self.declare_parameter('stationary_hold_s', 2.0)
        self.declare_parameter('target_detection_timeout_s', 0.5)
        self.declare_parameter('target_mask_radius_m', 0.30)
        # 로봇 외곽 0.565x0.42m 의 반대각은 0.352m 다. 이 반경이
        # 실제 외곽보다 작으면 오검출된 로봇의 모서리를 덮지 못하고, 팽창 후
        # A* 시작점을 막을 수 있다. 폭 실측(2026-08-28: 0.42m) 반영.
        self.declare_parameter('robot_mask_radius_m', 0.37)
        # 운반 차량 feedback은 YOLO 결과 순서가 아니라 Front/Rear 중점에
        # 가장 가까운 마스크만 사용한다.
        self.declare_parameter('vehicle_feedback_association_gate_m', 0.45)
        # 한 프레임 검출 누락을 빈자리로 오판하지 않는 안전 debounce.
        self.declare_parameter('slot_empty_confirm_frames', 5)
        self.declare_parameter('slot_occupied_hold_s', 0.75)
        # 바닥 Homography로 차량 상면 중심을 변환할 때 생기는 시차 보정.
        self.declare_parameter('camera_ground_x_m', 0.0)
        self.declare_parameter('camera_ground_y_m', 0.0)
        self.declare_parameter('camera_height_m', 0.0)
        self.declare_parameter('vehicle_detection_height_m', 0.0)
        # Segmentation mask에서 계산한 차량 외곽 치수를 vehicle_spec에 넣는다.
        # 상면 마스크가 실제 범퍼 외곽보다 작을 수 있어 사방 padding을 더한다.
        self.declare_parameter('use_mask_vehicle_dimensions', True)
        self.declare_parameter('default_vehicle_length_m', 0.90)
        self.declare_parameter('default_vehicle_width_m', 0.35)
        self.declare_parameter('vehicle_dimension_padding_m', 0.03)
        self.declare_parameter('vehicle_length_range_m', [0.30, 6.50])
        self.declare_parameter('vehicle_width_range_m', [0.20, 2.80])
        self.declare_parameter('vehicle_dimension_ema_alpha', 0.20)
        # 대기공간 영역 [x1, y1, x2, y2] m
        self.declare_parameter('waiting_zone', [2.10, 0.30, 2.50, 0.90])
        self.declare_parameter(
            'waiting_polygon',
            [2.10, 0.30, 2.50, 0.30, 2.50, 0.90, 2.10, 0.90])
        # 주차 슬롯은 같은 index의 ID/중심/길이·폭/통로->안쪽 Yaw로 구성한다.
        self.declare_parameter('slot_ids', ['P1', 'P2', 'P3', 'P4'])
        self.declare_parameter('slot_coords',
                               [1.5, 3.5, 2.5, 3.5, 3.5, 3.5, 4.5, 3.5])
        self.declare_parameter(
            'slot_sizes',
            [1.80, 0.70, 1.80, 0.70, 1.80, 0.70, 1.80, 0.70])
        self.declare_parameter('slot_yaws_deg', [90.0, 90.0, 90.0, 90.0])
        # 1개 sentinel(0.0)은 구버전 layout 호환용. 새 등록 파일은 슬롯당
        # 실제 클릭 모서리 8개(x,y x4)를 저장한다.
        self.declare_parameter('slot_polygons', [0.0])

        # ===== v1.11 듀얼 카메라 sensor 모드 =====
        # camera_id: 병합 노드가 카메라를 구분하는 이름. 배선 실수를 잡기 위해
        # merge 노드의 camera_ids와 반드시 같아야 한다.
        self.declare_parameter('camera_id', 'cam0')
        # true면 자기 검출 목록을 detection_topic으로 발행한다.
        self.declare_parameter('publish_detections', False)
        self.declare_parameter('detection_topic', '')
        # false면 /parking/* 임무 토픽을 직접 발행하지 않는다(병합 노드가 담당).
        self.declare_parameter('publish_mission_outputs', True)
        # coverage polygon은 H로 영상 네 귀퉁이를 투영해 자동 계산한다.
        # 테두리 픽셀은 왜곡 잔차와 검출 신뢰도가 낮아 잘라낸다.
        self.declare_parameter('coverage_margin_px', 8.0)

        self.homography_scale_to_m = float(
            self.get_parameter('homography_scale_to_m').value)
        if self.homography_scale_to_m <= 0.0:
            raise ValueError('homography_scale_to_m must be positive')
        self.resolution = float(self.get_parameter('map_resolution').value)
        self.map_w_m = float(self.get_parameter('map_width_m').value)
        self.map_h_m = float(self.get_parameter('map_height_m').value)
        self.conf = float(self.get_parameter('confidence').value)
        self.model_mode = normalize_model_mode(
            self.get_parameter('model_mode').value)
        if (bool(self.get_parameter('require_registered_layout').value) and
                not bool(self.get_parameter('layout_registered').value)):
            raise RuntimeError(
                '현장 등록 layout이 아닙니다; '
                'bev_layout_calibration.launch.py를 먼저 실행하세요')
        self.coco_vehicle_ids = parse_class_ids(
            self.get_parameter('coco_vehicle_class_ids').value)
        self.inference_imgsz = int(
            self.get_parameter('inference_imgsz').value)
        self.process_every_n = int(
            self.get_parameter('process_every_n').value)
        if self.inference_imgsz <= 0 or self.process_every_n <= 0:
            raise ValueError('inference_imgsz and process_every_n must be positive')
        self.frame_count = 0
        self.cls_vehicle = int(self.get_parameter('cls_vehicle').value)
        self.cls_empty = int(self.get_parameter('cls_empty_slot').value)
        if self.cls_vehicle < 0 or self.cls_empty < 0:
            raise ValueError('YOLO class IDs must be non-negative')
        if (self.model_mode == 'parking_seg' and
                self.cls_vehicle == self.cls_empty):
            raise ValueError('vehicle and empty-slot class IDs must differ')
        self.use_fixed_slots = bool(
            self.get_parameter('use_fixed_slots').value)
        self.slot_overlap_threshold = float(
            self.get_parameter('slot_occupancy_overlap_ratio').value)
        if not 0.0 <= self.slot_overlap_threshold <= 1.0:
            raise ValueError('slot_occupancy_overlap_ratio must be in [0,1]')
        self.car_size = float(self.get_parameter('car_size_m').value)
        self.feedback_association_gate = float(
            self.get_parameter('vehicle_feedback_association_gate_m').value)
        self.slot_empty_confirm_frames = int(
            self.get_parameter('slot_empty_confirm_frames').value)
        self.slot_occupied_hold = float(
            self.get_parameter('slot_occupied_hold_s').value)
        if (self.car_size <= 0.0 or self.feedback_association_gate <= 0.0 or
                self.slot_empty_confirm_frames <= 0 or
                self.slot_occupied_hold < 0.0):
            raise ValueError('invalid obstacle/slot debounce parameters')
        self.waiting_zone = list(self.get_parameter('waiting_zone').value)
        waiting_flat = list(self.get_parameter('waiting_polygon').value)
        if len(waiting_flat) != 8:
            raise ValueError('waiting_polygon must contain four x,y points')
        self.waiting_polygon = [
            (float(waiting_flat[index]), float(waiting_flat[index + 1]))
            for index in range(0, 8, 2)]
        if self.resolution <= 0.0 or self.map_w_m <= 0.0 or self.map_h_m <= 0.0:
            raise ValueError('map resolution/width/height must be positive')
        if len(self.waiting_zone) != 4:
            raise ValueError('waiting_zone must be [x1,y1,x2,y2]')
        if not (0.0 < self.conf <= 1.0):
            raise ValueError('confidence must be in (0,1]')
        self.grid_w = int(math.ceil(self.map_w_m / self.resolution))
        self.grid_h = int(math.ceil(self.map_h_m / self.resolution))
        self.stationary_tol = float(
            self.get_parameter('stationary_tolerance_m').value)
        self.stationary_hold = float(
            self.get_parameter('stationary_hold_s').value)
        self.target_detection_timeout = float(
            self.get_parameter('target_detection_timeout_s').value)
        self.target_mask_radius = float(
            self.get_parameter('target_mask_radius_m').value)
        self.robot_mask_radius = float(
            self.get_parameter('robot_mask_radius_m').value)
        self.camera_ground = (
            float(self.get_parameter('camera_ground_x_m').value),
            float(self.get_parameter('camera_ground_y_m').value),
        )
        self.camera_height = float(
            self.get_parameter('camera_height_m').value)
        self.vehicle_detection_height = float(
            self.get_parameter('vehicle_detection_height_m').value)
        if self.vehicle_detection_height < 0.0:
            raise ValueError('vehicle_detection_height_m must be non-negative')
        if (self.vehicle_detection_height > 0.0 and
                self.camera_height <= self.vehicle_detection_height):
            raise ValueError(
                'camera_height_m must exceed vehicle_detection_height_m')
        self.use_mask_vehicle_dimensions = bool(
            self.get_parameter('use_mask_vehicle_dimensions').value)
        self.default_vehicle_length = float(
            self.get_parameter('default_vehicle_length_m').value)
        self.default_vehicle_width = float(
            self.get_parameter('default_vehicle_width_m').value)
        self.vehicle_length = self.default_vehicle_length
        self.vehicle_width = self.default_vehicle_width
        # 첫 정상 mask가 들어오기 전에는 설정 기본값을 사용한다. 첫 측정부터
        # 기본값과 EMA를 섞으면 실제보다 작은/큰 치수가 오래 남으므로 별도 flag로
        # 구분하고, 첫 정상값은 그대로 채택한 뒤 두 번째부터 EMA를 적용한다.
        self.vehicle_dimension_valid = False
        self.vehicle_dimension_padding = float(
            self.get_parameter('vehicle_dimension_padding_m').value)
        self.vehicle_length_range = list(
            self.get_parameter('vehicle_length_range_m').value)
        self.vehicle_width_range = list(
            self.get_parameter('vehicle_width_range_m').value)
        self.vehicle_dimension_alpha = float(
            self.get_parameter('vehicle_dimension_ema_alpha').value)
        if (self.vehicle_length <= 0.0 or self.vehicle_width <= 0.0 or
                self.vehicle_dimension_padding < 0.0 or
                len(self.vehicle_length_range) != 2 or
                len(self.vehicle_width_range) != 2 or
                not 0.0 < self.vehicle_dimension_alpha <= 1.0):
            raise ValueError('invalid vehicle dimension parameters')
        if (self.vehicle_length_range[0] <= 0.0 or
                self.vehicle_length_range[1] < self.vehicle_length_range[0] or
                self.vehicle_width_range[0] <= 0.0 or
                self.vehicle_width_range[1] < self.vehicle_width_range[0]):
            raise ValueError('invalid vehicle length/width ranges')
        self.use_fixed_wheelbase = bool(
            self.get_parameter('use_fixed_wheelbase').value)
        self.fixed_wheelbase = float(
            self.get_parameter('fixed_wheelbase_m').value)
        if self.fixed_wheelbase <= 0.0:
            raise ValueError('fixed_wheelbase_m must be positive')
        self.target_candidate = None
        self.target_anchor = None
        self.target_stable_since = None
        self.target_last_seen = 0.0
        self.target_latched = None
        self.target_yaw = 0.0
        self.target_yaw_valid = False
        self.yaw_min_ratio = float(
            self.get_parameter('yaw_pca_min_ratio').value)
        self.yaw_alpha = float(
            self.get_parameter('yaw_ema_alpha').value)
        self.waiting_yaw = math.radians(float(
            self.get_parameter('waiting_yaw_deg').value))
        self.yaw_limit = math.radians(
            float(self.get_parameter('yaw_limit_deg').value))
        # --- 듀얼 카메라 sensor 모드 상태 ---
        self.camera_id = str(self.get_parameter('camera_id').value).strip()
        if not self.camera_id:
            raise ValueError('camera_id must not be empty')
        self.publish_detections = bool(
            self.get_parameter('publish_detections').value)
        self.publish_mission_outputs = bool(
            self.get_parameter('publish_mission_outputs').value)
        if not self.publish_detections and not self.publish_mission_outputs:
            raise ValueError(
                'publish_detections와 publish_mission_outputs가 모두 false면 '
                '이 노드는 아무 것도 발행하지 않습니다')
        self.coverage_margin_px = float(
            self.get_parameter('coverage_margin_px').value)
        if self.coverage_margin_px < 0.0:
            raise ValueError('coverage_margin_px must be non-negative')
        self.detection_sequence = 0
        # 영상 크기가 바뀌기 전까지 재사용하는 coverage 캐시.
        self._coverage_polygon = None
        self._coverage_size = None
        # 광축 지상점. camera_ground_*가 (0,0) 기본값이면 coverage 중심으로
        # 대체한다 — merge 노드의 중복 선택 기준(axis_dist_m)이 항상 의미
        # 있는 값을 갖게 하기 위해서다.
        self._axis_reference = None
        if self.yaw_min_ratio <= 1.0:
            raise ValueError('yaw_pca_min_ratio must exceed 1.0')
        if not 0.0 < self.yaw_alpha <= 1.0:
            raise ValueError('yaw_ema_alpha must be in (0,1]')
        if not 0.0 < self.yaw_limit <= math.pi / 2.0:
            raise ValueError('yaw_limit_deg must be in (0,90]')
        self.robot_pose = {'front': None, 'rear': None}

        # 브라우저 등록 도구가 만든 평탄 배열을 공통 ParkingSlot 객체로 복원한다.
        self.slots = parse_registered_slots(
            self.get_parameter('slot_ids').value,
            self.get_parameter('slot_coords').value,
            self.get_parameter('slot_sizes').value,
            self.get_parameter('slot_yaws_deg').value)
        if not self.slots:
            raise ValueError('at least one registered parking slot is required')
        polygon_flat = list(self.get_parameter('slot_polygons').value)
        if len(polygon_flat) == 1 and float(polygon_flat[0]) == 0.0:
            polygons = [slot_polygon(slot) for slot in self.slots]
            self.get_logger().warn(
                'slot_polygons 미등록 — fitted rectangle 호환 폴백')
        elif len(polygon_flat) == 8 * len(self.slots):
            polygons = [
                tuple((
                    float(polygon_flat[8 * index + 2 * corner]),
                    float(polygon_flat[8 * index + 2 * corner + 1]))
                    for corner in range(4))
                for index in range(len(self.slots))
            ]
        else:
            raise ValueError('slot_polygons must contain eight values per slot')
        self.slot_polygons = {
            slot.slot_id: polygon
            for slot, polygon in zip(self.slots, polygons)
        }
        now = time.monotonic()
        self.slot_occupancy_state = {
            slot.slot_id: {
                'occupied': True,  # 시작 상태는 빈자리가 아니라 unknown/점유.
                'empty_count': 0,
                'last_occupied': now,
            }
            for slot in self.slots
        }

        # ===== 모델 로드 =====
        self.bridge = CvBridge() if DEPS_OK else None
        self.model = None
        self.H = None
        if not DEPS_OK and self.get_parameter('require_dependencies').value:
            raise RuntimeError(
                'OpenCV/cv_bridge/ultralytics dependencies are required')
        if DEPS_OK:
            self._load_models()

        # ===== 구독 =====
        self.image_topic = str(self.get_parameter('image_topic').value)
        if not self.image_topic:
            raise ValueError('image_topic must not be empty')
        self.create_subscription(Image, self.image_topic,
                                 self.image_cb, SENSOR_LATEST_QOS)
        for role in ('front', 'rear'):
            self.create_subscription(
                Odometry, f'/{role}/odom',
                lambda msg, r=role: self.odom_cb(r, msg),
                SENSOR_LATEST_QOS)
        # P3: 임무가 끝나면 타겟 latch를 풀어 다음 차량을 인식할 수 있게 한다.
        self.create_subscription(
            String, '/mission/complete', self.mission_complete_cb, 10)

        # ===== 발행 =====
        self.mission_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # publish_mission_outputs=false(듀얼 카메라 sensor 모드)일 때는 임무
        # 토픽 publisher 자체를 만들지 않는다. 두 인스턴스가 같은 토픽에
        # publisher를 열어두면 `ros2 topic info`로 봐도 누가 진짜인지 알 수
        # 없고, 실수로 발행되는 순간 fleet가 카메라별로 엇갈린 맵을 받는다.
        if self.publish_mission_outputs:
            self.pub_map = self.create_publisher(
                OccupancyGrid, '/parking/map', 10)
            self.pub_target = self.create_publisher(
                PoseStamped, '/parking/target_pose', 10)
            self.pub_empty = self.create_publisher(
                PoseArray, '/parking/empty_slots', 10)
            # 차종 제원 발행 (individual_move가 휠베이스 사용)
            self.pub_spec = self.create_publisher(
                String, '/parking/vehicle_spec', self.mission_qos)
            # 협조주행 중 든 차량 실위치 피드백 (rigid_body_sync 절대보정용)
            self.pub_vehicle_fb = self.create_publisher(
                PoseStamped, '/parking/vehicle_pose_feedback',
                SENSOR_LATEST_QOS)
            self.pub_target_ready = self.create_publisher(
                Bool, '/parking/target_ready', 10)
            # 맵은 주기적으로도 발행
            self.create_timer(1.0, self.publish_map_periodic)
        else:
            self.pub_map = None
            self.pub_target = None
            self.pub_empty = None
            self.pub_spec = None
            self.pub_vehicle_fb = None
            self.pub_target_ready = None

        self.pub_detections = None
        if self.publish_detections:
            detection_topic = str(
                self.get_parameter('detection_topic').value).strip()
            if not detection_topic:
                detection_topic = f'/{self.camera_id}/detections'
            self.detection_topic = detection_topic
            self.pub_detections = self.create_publisher(
                String, detection_topic, SENSOR_LATEST_QOS)
        else:
            self.detection_topic = ''

        self.latest_obstacles = []

        self.get_logger().info(
            'yolo_bev_map 시작 | '
            f'camera_id={self.camera_id} | '
            f'wheelbase_mode={"fixed" if self.use_fixed_wheelbase else "classified"} | '
            f'fixed={self.fixed_wheelbase:.3f}m | mode={self.model_mode} | '
            f'image={self.image_topic} | '
            f'mission_outputs={self.publish_mission_outputs} | '
            f'detections={self.detection_topic or "off"}')

    def _load_models(self):
        mp = os.path.expanduser(str(self.get_parameter('model_path').value))
        if not mp:
            raise ValueError('model_path must not be empty')
        # Ultralytics는 알려진 모델 이름이 로컬에 없으면 인터넷에서 자동으로
        # 내려받을 수 있다. 실차는 재현 가능한 오프라인 자산만 허용하므로
        # YOLO 생성자 호출 전에 모든 모드에서 실제 로컬 파일을 요구한다.
        if not os.path.isfile(mp):
            raise FileNotFoundError(
                f'YOLO requires a local model file; network download is '
                f'disabled: {mp}')
        try:
            # `.engine`/`.onnx` 에는 task 정보가 없다. YOLO(mp) 로만 열면
            # ultralytics 가 task=detect 로 추정해 **mask 가 사라진다**.
            # mask 가 없으면 차량 길이/폭을 못 내고 vehicle_spec 이 영원히
            # invalid 라서 미션이 시작되지 않는다. load_yolo_model 이
            # model_mode 에 맞는 task 를 붙여준다.
            self.model, _task = load_yolo_model(YOLO, mp, self.model_mode)
            self._validate_model_classes()
            self.get_logger().info(
                f'YOLO loaded: {mp} | mode={self.model_mode} | '
                f'imgsz={self.inference_imgsz} every_n={self.process_every_n}')
        except Exception as e:
            raise RuntimeError(f'YOLO model load failed: {mp}') from e

        self.classifier = None
        cp = self.get_parameter('classifier_path').value
        if os.path.exists(cp):
            try:
                import torch
                self.classifier = torch.load(cp, map_location='cpu')
                self.classifier.eval()
                self.get_logger().info(f'차종 분류기 로드: {cp}')
            except Exception as e:
                self.get_logger().warn(f'분류기 로드 실패: {e}')
        hf = os.path.expanduser(str(
            self.get_parameter('homography_file').value))
        if os.path.exists(hf):
            self.H = np.load(hf)
            if self.H.shape != (3, 3) or not np.all(np.isfinite(self.H)):
                raise ValueError('homography matrix must be finite 3x3')
            if abs(np.linalg.det(self.H)) < 1e-12:
                raise ValueError('homography matrix is singular')
            self.get_logger().info('호모그래피 로드')
        else:
            if self.get_parameter('require_homography').value:
                raise FileNotFoundError(
                    f'homography missing: {hf}; run calibration first')
            self.get_logger().warn(f'{hf} 없음 — 캘리브레이션 필요')

    def _validate_model_classes(self):
        """Fail early when configured class IDs do not exist in the model."""
        if self.model is None:
            return
        names = getattr(self.model, 'names', None)
        if names is None:
            self.get_logger().warn(
                'YOLO model class names 확인 불가 — 첫 추론 전 class ID 검증 필요')
            return

        def lookup(index):
            if isinstance(names, dict):
                return names.get(index, names.get(str(index)))
            if isinstance(names, (list, tuple)) and 0 <= index < len(names):
                return names[index]
            return None

        if self.model_mode in ('vehicle_seg', 'parking_seg'):
            vehicle_name = lookup(self.cls_vehicle)
            if vehicle_name is None:
                raise ValueError(
                    'vehicle segmentation class mapping mismatch: '
                    f'vehicle={self.cls_vehicle}, '
                    f'names={names}')
            if self.model_mode == 'vehicle_seg':
                self.get_logger().info(
                    f'vehicle-only model class: {self.cls_vehicle}={vehicle_name}')
                return
            empty_name = lookup(self.cls_empty)
            if empty_name is None:
                raise ValueError(
                    'parking_seg empty class mapping mismatch: '
                    f'empty={self.cls_empty}, names={names}')
            self.get_logger().info(
                'parking model class mapping: '
                f'{self.cls_vehicle}={vehicle_name}, '
                f'{self.cls_empty}={empty_name}')
            return

        missing = [
            class_id for class_id in self.coco_vehicle_ids
            if lookup(class_id) is None]
        if missing:
            raise ValueError(
                f'COCO vehicle class IDs missing from model: {missing}')
        labels = ', '.join(
            f'{class_id}={lookup(class_id)}'
            for class_id in self.coco_vehicle_ids)
        self.get_logger().info(f'COCO vehicle classes: {labels}')

    def mission_complete_cb(self, msg):
        """임무 종료 — 타겟 latch와 차량 제원 캐시를 모두 해제한다.

        target_latched가 남아 있으면 다음 차량이 대기공간에 들어와도 새 타겟을
        잡지 못하고, _spec_sent가 남아 있으면 vehicle_spec이 재발행되지 않아
        Fleet이 이전 차량 제원으로 계획한다.
        """
        import json as _json
        try:
            payload = _json.loads(msg.data)
            mission_id = str(payload['mission_id'])
            stamp_ns = int(payload['stamp_ns'])
        except (KeyError, TypeError, ValueError, _json.JSONDecodeError):
            self.get_logger().warn('invalid mission/complete envelope')
            return
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        if not -0.5 <= age_s <= 10.0:
            return
        if self.target_latched is None:
            return
        self.target_latched = None
        self.target_candidate = None
        self.target_anchor = None
        self.target_stable_since = None
        self.target_last_seen = 0.0
        self.target_yaw = 0.0
        self.target_yaw_valid = False
        self.vehicle_length = self.default_vehicle_length
        self.vehicle_width = self.default_vehicle_width
        self.vehicle_dimension_valid = False
        self._spec_sent = False
        if self.pub_target_ready is not None:
            self.pub_target_ready.publish(Bool(data=False))
        self.get_logger().info(
            f'임무 {mission_id} 완료 — 타겟 latch 해제')

    def odom_cb(self, role, msg):
        self.robot_pose[role] = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y))

    def pixel_to_world(self, px, py):
        if self.H is None:
            return None, None
        pt = np.array([px, py, 1.0])
        r = self.H @ pt
        if abs(r[2]) < 1e-10:
            return None, None
        return (
            r[0] / r[2] * self.homography_scale_to_m,
            r[1] / r[2] * self.homography_scale_to_m,
        )

    def correct_vehicle_parallax(self, floor_x, floor_y):
        """Correct a floor-plane ray intersection to the vehicle detection plane."""
        return correct_floor_projection(
            floor_x, floor_y,
            self.camera_ground[0], self.camera_ground[1],
            self.camera_height, self.vehicle_detection_height)

    # ------------------------------------------------------------------
    # 듀얼 카메라 sensor 모드 helper
    # ------------------------------------------------------------------
    def ensure_coverage_polygon(self, width_px, height_px):
        """이 카메라가 바닥의 어디를 보는지(coverage polygon)를 H에서 유도한다.

        사람이 자로 재서 파라미터로 넣으면 H와 반드시 어긋나므로, 영상 네
        귀퉁이를 같은 H로 투영해서 만든다. 영상 크기가 그대로면 캐시를 쓴다.
        """
        if self.H is None:
            return None
        if self._coverage_size == (width_px, height_px):
            return self._coverage_polygon
        try:
            polygon = image_corner_coverage(
                self.H, width_px, height_px,
                margin_px=self.coverage_margin_px,
                scale_to_m=self.homography_scale_to_m)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                f'coverage polygon 계산 실패: {exc}',
                throttle_duration_sec=10.0)
            return None
        self._coverage_polygon = polygon
        self._coverage_size = (width_px, height_px)
        # 광축 지상점 기준. 실측 camera_ground_*가 (0,0)이면 coverage 중심을
        # 대신 쓴다. 두 카메라가 같은 물체를 봤을 때 "누가 더 광축에
        # 가까운가"를 비교할 수 있으면 되므로 절대 정확도는 필요 없다.
        if self.camera_ground[0] != 0.0 or self.camera_ground[1] != 0.0:
            self._axis_reference = (self.camera_ground[0], self.camera_ground[1])
        else:
            self._axis_reference = (
                sum(point[0] for point in polygon) / len(polygon),
                sum(point[1] for point in polygon) / len(polygon))
            self.get_logger().warn(
                'camera_ground_x/y_m 미실측 — coverage 중심을 광축 근사로 사용',
                throttle_duration_sec=60.0)
        self.get_logger().info(
            f'[{self.camera_id}] coverage polygon: ' +
            ', '.join(f'({x:.2f},{y:.2f})' for x, y in polygon))
        return self._coverage_polygon

    def axis_distance_m(self, world_x, world_y):
        """광축 지상점에서 이 검출까지의 거리. parallax 오차의 대리 지표."""
        if self._axis_reference is None:
            return 0.0
        return math.hypot(world_x - self._axis_reference[0],
                          world_y - self._axis_reference[1])

    def publish_detection_envelope(self, detections, stamp, width_px,
                                   height_px):
        """이 카메라의 프레임 결과를 병합 노드로 보낸다."""
        if self.pub_detections is None:
            return
        coverage = self.ensure_coverage_polygon(width_px, height_px)
        self.detection_sequence += 1
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns == 0:
            stamp_ns = self.get_clock().now().nanoseconds
        message = String()
        message.data = encode_detection_envelope(
            camera_id=self.camera_id,
            stamp_ns=stamp_ns,
            sequence=self.detection_sequence,
            coverage_polygon=coverage,
            detections=detections,
            homography_ok=self.H is not None)
        self.pub_detections.publish(message)

    def in_waiting_zone(self, x, y):
        return self.point_in_polygon(x, y, self.waiting_polygon)

    @staticmethod
    def point_in_polygon(x, y, polygon):
        """경계 포함 ray-casting. COCO box 중심 fallback에도 같은 함수를 쓴다."""
        inside = False
        count = len(polygon)
        for index, (x1, y1) in enumerate(polygon):
            x2, y2 = polygon[(index + 1) % count]
            cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
            if (abs(cross) <= 1e-9 and
                    min(x1, x2) - 1e-9 <= x <= max(x1, x2) + 1e-9 and
                    min(y1, y2) - 1e-9 <= y <= max(y1, y2) + 1e-9):
                return True
            if ((y1 > y) != (y2 > y)):
                hit_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                if x < hit_x:
                    inside = not inside
        return inside

    # ================================================
    # 이미지 콜백 — 메인 처리
    # ================================================
    def image_cb(self, msg):
        if self.model is None:
            return
        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(
            frame, conf=self.conf, imgsz=self.inference_imgsz, verbose=False)

        obstacles = []        # 맵용 (모든 차량 위치)
        cars_in_slots = []    # {'center': (x,y), 'polygon': convex hull, 'yaw': ...}
        detected_empty = []   # YOLO가 직접 검출한 빈자리 (커스텀 모델)
        target = None         # 대기공간 차량
        target_crop = None    # 차종 분류용 crop
        target_geometry = None  # (yaw, length, width) — 차량 mask 기반
        # 듀얼 카메라 sensor 모드에서 병합 노드로 보낼 카메라별 검출 목록.
        sensor_detections = []
        frame_height, frame_width = frame.shape[:2]

        cls_vehicle = self.cls_vehicle
        cls_empty = self.cls_empty

        for result in results:
            if result.boxes is None:
                continue
            masks_xy = None
            if getattr(result, 'masks', None) is not None:
                masks_xy = getattr(result.masks, 'xy', None)
            for box_index, box in enumerate(result.boxes):
                cls = int(box.cls[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx, cy = (x1+x2)/2, (y1+y2)/2
                floor_x, floor_y = self.pixel_to_world(cx, cy)
                if floor_x is None:
                    continue
                mask_xy = (
                    masks_xy[box_index]
                    if masks_xy is not None and box_index < len(masks_xy)
                    else None)
                try:
                    confidence = float(box.conf[0])
                except (AttributeError, IndexError, TypeError, ValueError):
                    confidence = 0.0

                if self.model_mode in ('vehicle_seg', 'parking_seg'):
                    # 차량 전용 모델 또는 기존 vehicle/empty_slot 2클래스 모델.
                    if cls == cls_vehicle:
                        wx, wy = self.correct_vehicle_parallax(floor_x, floor_y)
                        world_polygon = self.mask_world_polygon(mask_xy)
                        geometry = self.mask_geometry(world_polygon)
                        obstacles.append({
                            'center': (wx, wy), 'polygon': world_polygon})
                        in_waiting = self.in_waiting_zone(wx, wy)
                        sensor_detections.append(self._make_sensor_detection(
                            wx, wy, world_polygon, geometry, in_waiting,
                            confidence))
                        if in_waiting:
                            target = (wx, wy)
                            target_crop = frame[int(y1):int(y2),
                                                int(x1):int(x2)]
                            target_geometry = geometry
                        else:
                            cars_in_slots.append({
                                'center': (wx, wy),
                                'polygon': world_polygon,
                                'yaw': None if geometry is None else geometry[0],
                            })
                    elif self.model_mode == 'parking_seg' and cls == cls_empty:
                        # empty_slot은 바닥이므로 차량 높이 parallax를 적용하면 안 된다.
                        detected_empty.append((floor_x, floor_y))
                else:
                    # COCO 폴백: car(2)/truck(7)만 차량으로
                    if cls not in self.coco_vehicle_ids:
                        continue
                    wx, wy = self.correct_vehicle_parallax(floor_x, floor_y)
                    obstacles.append({
                        'center': (wx, wy), 'polygon': None})
                    in_waiting = self.in_waiting_zone(wx, wy)
                    sensor_detections.append(self._make_sensor_detection(
                        wx, wy, None, None, in_waiting, confidence))
                    if in_waiting:
                        target = (wx, wy)
                        target_crop = frame[int(y1):int(y2),
                                            int(x1):int(x2)]
                    else:
                        cars_in_slots.append({
                            'center': (wx, wy), 'polygon': None, 'yaw': None})

        # sensor 모드: 자기 관측만 그대로 내보낸다. 아래 latch/슬롯/맵 로직은
        # publish_mission_outputs=true인 단일 카메라 구성에서만 의미가 있다.
        if self.publish_detections:
            self.publish_detection_envelope(
                sensor_detections, msg.header.stamp, frame_width, frame_height)
        if not self.publish_mission_outputs:
            # 분류기는 원본 crop이 필요하므로 sensor 모드에서도 여기서 돌린다.
            # 결과는 다음 프레임 envelope의 vehicle_class 필드로 실려 나간다.
            if target_crop is not None:
                self.classify_vehicle(target_crop)
            self.latest_obstacles = obstacles
            return

        if target_geometry is not None:
            yaw, length_m, width_m = target_geometry
            self.update_target_yaw(yaw)
            self.update_vehicle_dimensions(length_m, width_m)
        ready_target = self.update_target_latch(target)

        # 정차가 2초 확인된 타겟만 제원/상태기계에 전달한다.
        if ready_target is not None and target_crop is not None:
            self.classify_vehicle(target_crop)

        if ready_target is not None:
            self.publish_target(*ready_target)

        # 협조주행 중 차량 추적: 대기공간 밖에서 움직이는 차량을
        # 든 차량으로 간주하고 실위치 피드백 발행
        associated_car = self.associate_transported_vehicle(cars_in_slots)
        if not target and associated_car is not None:
            vx_, vy_ = associated_car['center']
            fb = PoseStamped()
            # 처리 지연을 숨기지 않도록 원본 CCTV 촬영시각을 전파한다.
            fb.header.stamp = msg.header.stamp
            fb.header.frame_id = 'map'
            fb.pose.position.x = vx_
            fb.pose.position.y = vy_
            feedback_yaw = associated_car['yaw']
            if feedback_yaw is None:
                fb.pose.orientation.w = 1.0
            else:
                fb.pose.orientation.z = math.sin(feedback_yaw / 2.0)
                fb.pose.orientation.w = math.cos(feedback_yaw / 2.0)
            self.pub_vehicle_fb.publish(fb)

        # 빈자리 판별 + 발행
        # 새 기본은 차량 mask와 고정 슬롯 DB의 면적 겹침률이다. 기존 2클래스
        # 모델을 유지해야 할 때만 use_fixed_slots=false로 empty_slot 검출을 쓴다.
        if self.model_mode == 'parking_seg' and not self.use_fixed_slots:
            # '0개 검출'도 유효한 결과다. 이때 DB fallback을 하면 실제로
            # 만차인데 빈자리를 만들어내므로 빈 PoseArray를 그대로 발행한다.
            self.publish_detected_empty(detected_empty)
        else:
            self.publish_empty_slots(cars_in_slots)

        # 운반 대상은 A* 장애물에서 제거해 시작점이 막히지 않게 한다.
        if self.target_latched is not None:
            tx, ty = self.target_latched
            obstacles = [
                obstacle for obstacle in obstacles
                if math.hypot(
                    obstacle['center'][0] - tx,
                    obstacle['center'][1] - ty) >
                self.target_mask_radius
            ]
        self.latest_obstacles = obstacles

    def _make_sensor_detection(self, wx, wy, world_polygon, geometry,
                               in_waiting, confidence):
        """병합 노드로 보낼 CameraDetection 하나를 만든다.

        차종 분류 결과를 여기 같이 실어야 하는 이유: 분류기는 원본 crop
        이미지가 필요하므로 카메라 노드에서만 돌릴 수 있다. 병합 노드는
        이미지를 보지 않으므로 이 필드가 없으면 wheelbase가 항상 기본값이 된다.
        """
        yaw = None if geometry is None else geometry[0]
        length_m = None if geometry is None else geometry[1]
        width_m = None if geometry is None else geometry[2]
        return CameraDetection(
            camera_id=self.camera_id,
            center=(wx, wy),
            polygon=world_polygon,
            yaw=yaw,
            length_m=length_m,
            width_m=width_m,
            in_waiting=bool(in_waiting),
            confidence=float(confidence),
            axis_dist_m=self.axis_distance_m(wx, wy),
            vehicle_class=getattr(self, '_classified_class', None),
            classified_wheelbase_m=getattr(
                self, '_classified_wheelbase', None),
        )

    def associate_transported_vehicle(self, cars):
        """Front/Rear 중점 예측치와 gate를 통과한 차량 mask만 반환한다."""
        front = self.robot_pose.get('front')
        rear = self.robot_pose.get('rear')
        if not cars or front is None or rear is None:
            return None
        predicted = ((front[0] + rear[0]) / 2.0,
                     (front[1] + rear[1]) / 2.0)
        candidate = min(
            cars,
            key=lambda car: math.hypot(
                car['center'][0] - predicted[0],
                car['center'][1] - predicted[1]))
        distance = math.hypot(
            candidate['center'][0] - predicted[0],
            candidate['center'][1] - predicted[1])
        if distance > self.feedback_association_gate:
            self.get_logger().warn(
                f'운반 차량 feedback association gate 초과: {distance:.3f}m',
                throttle_duration_sec=2.0)
            return None
        return candidate

    def mask_world_polygon(self, mask_xy):
        """Seg mask contour를 map metre의 볼록 hull로 변환한다.

        Homography는 바닥 평면 기준이므로 차량 상면의 각 점에도 같은 parallax
        보정을 적용한다. 빈 슬롯/주차선에는 이 함수를 호출하지 않는다.
        """
        if mask_xy is None or len(mask_xy) < 3:
            return None
        points = []
        for px, py in mask_xy:
            wx, wy = self.pixel_to_world(float(px), float(py))
            if wx is not None:
                points.append(self.correct_vehicle_parallax(wx, wy))
        if len(points) < 3:
            return None
        hull = cv2.convexHull(
            np.asarray(points, dtype=np.float32)).reshape(-1, 2)
        return [(float(point[0]), float(point[1])) for point in hull]

    def mask_geometry(self, world_polygon):
        """차량 hull의 중심축 Yaw와 실제 길이/폭(m)을 계산한다."""
        if world_polygon is None or len(world_polygon) < 4:
            return None
        yaw = principal_axis_yaw(
            world_polygon, self.yaw_min_ratio, self.yaw_limit)
        if yaw is None:
            return None
        c, s = math.cos(yaw), math.sin(yaw)
        longitudinal = [x * c + y * s for x, y in world_polygon]
        lateral = [-x * s + y * c for x, y in world_polygon]
        length_m = max(longitudinal) - min(longitudinal)
        width_m = max(lateral) - min(lateral)
        if length_m < width_m:
            length_m, width_m = width_m, length_m
            yaw = math.atan2(math.sin(yaw + math.pi / 2.0),
                             math.cos(yaw + math.pi / 2.0))
        return yaw, length_m, width_m

    def update_vehicle_dimensions(self, measured_length, measured_width):
        """비정상 mask 치수는 버리고 유효한 값만 EMA로 vehicle_spec에 반영한다."""
        if not self.use_mask_vehicle_dimensions:
            return
        length = float(measured_length) + 2.0 * self.vehicle_dimension_padding
        width = float(measured_width) + 2.0 * self.vehicle_dimension_padding
        if not (self.vehicle_length_range[0] <= length <=
                self.vehicle_length_range[1]):
            return
        if not (self.vehicle_width_range[0] <= width <=
                self.vehicle_width_range[1]):
            return
        if not self.vehicle_dimension_valid:
            self.vehicle_length = length
            self.vehicle_width = width
            self.vehicle_dimension_valid = True
            return
        alpha = self.vehicle_dimension_alpha
        self.vehicle_length = (
            (1.0 - alpha) * self.vehicle_length + alpha * length)
        self.vehicle_width = (
            (1.0 - alpha) * self.vehicle_width + alpha * width)

    def update_target_yaw(self, yaw):
        if yaw is None:
            return
        if not self.target_yaw_valid:
            self.target_yaw = yaw
            self.target_yaw_valid = True
            return
        alpha = self.yaw_alpha
        # 차량 장축은 yaw와 yaw+pi가 같은 축이다. 2*yaw 벡터를 평균해야
        # +89°와 -89°가 들어왔을 때 0°로 잘못 평균되지 않는다.
        x = ((1.0 - alpha) * math.cos(2.0 * self.target_yaw) +
             alpha * math.cos(2.0 * yaw))
        y = ((1.0 - alpha) * math.sin(2.0 * self.target_yaw) +
             alpha * math.sin(2.0 * yaw))
        self.target_yaw = 0.5 * math.atan2(y, x)

    def update_target_latch(self, target):
        now = time.monotonic()
        if self.target_latched is not None:
            return self.target_latched
        if target is None:
            if now - self.target_last_seen > self.target_detection_timeout:
                self.target_candidate = None
                self.target_anchor = None
                self.target_stable_since = None
                self.target_yaw = 0.0
                self.target_yaw_valid = False
                self.vehicle_length = self.default_vehicle_length
                self.vehicle_width = self.default_vehicle_width
                self.vehicle_dimension_valid = False
            return None
        self.target_last_seen = now
        if self.target_anchor is None or math.hypot(
                target[0] - self.target_anchor[0],
                target[1] - self.target_anchor[1]) > self.stationary_tol:
            self.target_anchor = target
            self.target_candidate = target
            self.target_stable_since = now
            return None
        self.target_candidate = (
            0.8 * self.target_candidate[0] + 0.2 * target[0],
            0.8 * self.target_candidate[1] + 0.2 * target[1])
        if now - self.target_stable_since >= self.stationary_hold:
            self.target_latched = self.target_candidate
            self.pub_target_ready.publish(Bool(data=True))
            self.get_logger().info(
                '타겟 정차 확인(2cm/2s) — target_ready latch')
        return self.target_latched

    def classify_vehicle(self, crop):
        """차종은 분류할 수 있지만 휠베이스는 설정에 따라 고정한다."""
        if not hasattr(self, '_spec_sent'):
            self._spec_sent = False
        if self._spec_sent:
            return
        import json as _json

        vehicle_class = 'default'
        classified_wheelbase = self.fixed_wheelbase
        if self.classifier is not None and crop.size > 0:
            try:
                import torch
                import cv2 as _cv2
                img = _cv2.resize(crop, (224, 224))
                t = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
                with torch.no_grad():
                    out = self.classifier(t.unsqueeze(0))
                idx = int(out.argmax())
                spec_db = {0: ('sedan', 0.70), 1: ('suv', 0.75),
                           2: ('compact', 0.67)}
                vehicle_class, classified_wheelbase = spec_db.get(
                    idx, ('default', self.fixed_wheelbase))
            except Exception as exc:
                self.get_logger().warn(f'분류 실패: {exc}')

        # sensor 모드에서는 이 결과를 발행하지 않고 envelope으로 실어 보낸다.
        self._classified_class = vehicle_class
        self._classified_wheelbase = classified_wheelbase

        if self.use_fixed_wheelbase:
            wheelbase = self.fixed_wheelbase
            wheelbase_mode = 'fixed'
        else:
            wheelbase = classified_wheelbase
            wheelbase_mode = 'classified'

        if self.pub_spec is None:
            self._spec_sent = True
            self.get_logger().info(
                f'[{self.camera_id}] 차종={vehicle_class} '
                f'(sensor 모드 — vehicle_spec은 cctv_merge_node가 발행)')
            return

        msg = String()
        msg.data = _json.dumps({
            'class': vehicle_class,
            'wheelbase': wheelbase,
            'wheelbase_mode': wheelbase_mode,
            # Fleet Manager는 이 값으로 차량 단독이 아니라 로봇까지 포함한
            # loaded footprint를 만든 뒤 주차면 길이/폭과 비교한다.
            'vehicle_length_m': self.vehicle_length,
            'vehicle_width_m': self.vehicle_width,
            'dimension_source': (
                'segmentation_mask' if self.vehicle_dimension_valid
                else 'configured_default'),
            'dimension_valid': bool(self.vehicle_dimension_valid),
            'sequence': 1,
            'stamp_ns': self.get_clock().now().nanoseconds,
        })
        self.pub_spec.publish(msg)
        self._spec_sent = True
        self.get_logger().info(
            f'차종={vehicle_class}, wheelbase={wheelbase:.3f}m '
            f'({wheelbase_mode}), size={self.vehicle_length:.3f}x'
            f'{self.vehicle_width:.3f}m')

    def publish_detected_empty(self, empties):
        """YOLO가 직접 검출한 빈자리 발행 (커스텀 모델)"""
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'map'
        for ex, ey in empties:
            p = Pose()
            p.position.x = ex
            p.position.y = ey
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_empty.publish(pa)

    def publish_target(self, x, y):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        half_yaw = directed_axis_yaw(
            self.target_yaw, self.waiting_yaw) / 2.0
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)
        self.pub_target.publish(msg)

    def publish_empty_slots(self, cars):
        """고정 슬롯 점유를 debounce한 뒤 확정된 빈 Pose만 발행한다."""
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'map'
        for slot in self.slots:
            registered_polygon = self.slot_polygons[slot.slot_id]
            observed_occupied = False
            for car in cars:
                vehicle_polygon = car.get('polygon')
                if vehicle_polygon is not None:
                    overlap = polygon_overlap_ratio(
                        vehicle_polygon, registered_polygon)
                    if overlap >= self.slot_overlap_threshold:
                        observed_occupied = True
                        break
                else:
                    # COCO detection 모델에는 mask가 없으므로 중심 포함 여부만
                    # fallback으로 사용한다. 실제 운용은 vehicle_seg를 권장한다.
                    cx, cy = car['center']
                    if self.point_in_polygon(cx, cy, registered_polygon):
                        observed_occupied = True
                        break
            state = self.slot_occupancy_state[slot.slot_id]
            now = time.monotonic()
            if observed_occupied:
                state['occupied'] = True
                state['empty_count'] = 0
                state['last_occupied'] = now
            elif now - state['last_occupied'] < self.slot_occupied_hold:
                state['empty_count'] = 0
            else:
                state['empty_count'] += 1
                if state['empty_count'] >= self.slot_empty_confirm_frames:
                    state['occupied'] = False

            if not state['occupied']:
                p = Pose()
                p.position.x = slot.center_x_m
                p.position.y = slot.center_y_m
                p.orientation.z = math.sin(slot.entry_yaw_rad / 2.0)
                p.orientation.w = math.cos(slot.entry_yaw_rad / 2.0)
                pa.poses.append(p)
        self.pub_empty.publish(pa)

    def publish_map_periodic(self):
        grid = np.zeros((self.grid_h, self.grid_w), dtype=np.int8) \
            if DEPS_OK else None
        if grid is None:
            return
        # Seg 모드에서는 실제 world hull을 채워 긴 차량을 0.90m 정사각형으로
        # 축소하지 않는다. mask가 없는 COCO box만 car_size fallback을 쓴다.
        car_px = max(1, int(math.ceil(self.car_size / self.resolution)))
        for obstacle in self.latest_obstacles:
            polygon = obstacle.get('polygon')
            if polygon is not None and len(polygon) >= 3:
                contour = np.asarray([
                    [int(round(x / self.resolution)),
                     int(round(y / self.resolution))]
                    for x, y in polygon
                ], dtype=np.int32)
                cv2.fillPoly(grid, [contour], 100)
                continue
            wx, wy = obstacle['center']
            gx = int(wx / self.resolution)
            gy = int(wy / self.resolution)
            half = car_px // 2
            y1 = max(0, gy - half)
            y2 = min(self.grid_h, gy + half)
            x1 = max(0, gx - half)
            x2 = min(self.grid_w, gx + half)
            grid[y1:y2, x1:x2] = 100

        # YOLO가 로봇을 차량으로 오검출해도 시작점을 막지 않도록 self-mask.
        mask_cells = max(1, int(self.robot_mask_radius / self.resolution))
        for pose in self.robot_pose.values():
            if pose is None:
                continue
            gx = int(pose[0] / self.resolution)
            gy = int(pose[1] / self.resolution)
            y1 = max(0, gy - mask_cells)
            y2 = min(self.grid_h, gy + mask_cells + 1)
            x1 = max(0, gx - mask_cells)
            x2 = min(self.grid_w, gx + mask_cells + 1)
            grid[y1:y2, x1:x2] = 0

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_w
        msg.info.height = self.grid_h
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        self.pub_map.publish(msg)
        self.pub_target_ready.publish(
            Bool(data=self.target_latched is not None))


def main(args=None):
    rclpy.init(args=args)
    node = YoloBevMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
