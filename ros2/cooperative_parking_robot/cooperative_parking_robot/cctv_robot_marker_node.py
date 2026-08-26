#!/usr/bin/env python3
"""
==================================================
cctv_robot_marker_node.py (Jetson — CCTV 서버)
==================================================
천장 CCTV로 Front와 Rear 상판 마커를 읽어 두 로봇의
절대 위치·yaw 기준을 만든다.

localization_design.md §3, §10-1에 정의된 절대 pose 공급 노드다.
현재 구현된 이 노드가 있어야 pose_fusion_node의 correct()가 실제로
호출되고 /front/odom, /rear/odom이 이름값을 한다. 차량 아래에서 각
상판 마커가 가려지면 해당 pose_fusion은 엔코더 예측을 이어가고,
Front 후면 ID0 상대 pose가 두 로봇 사이 오차를 보완한다.

aruco_tracker_node(Rear 카메라, 측면, Front 후면 마커 ID0 관측)와는
완전히 다른 마커·카메라다. 천장용 marker ID는 Rear 카메라가 관측하는
후면 ID0과 겹치면 안 된다(기본값: front=10, rear=11).

입력:
  /cctv/image_rect (sensor_msgs/Image) — yolo_bev_map_node와 동일한 보정 영상
  (v1.11) image_topics/homography_files 파라미터로 천장 카메라 N대 동시 구독
출력:
  /front/cctv_pose (geometry_msgs/PoseStamped, frame_id='map')
  /front/cctv_marker_visible (std_msgs/Bool)
  /rear/cctv_pose (geometry_msgs/PoseStamped, frame_id='map')
  /rear/cctv_marker_visible (std_msgs/Bool)

듀얼 카메라 (v1.11, docs/DUAL_CCTV_MERGE_20260812.md)
----------------------------------------------------
천장 카메라를 2대 쓰더라도 **이 노드는 하나만** 띄운다. 노드를 카메라마다
띄우면 /front/cctv_pose에 publisher가 둘이 되고, pose_fusion_node의 EKF가
같은 프레임 정보로 두 번 correct하게 된다(관측 잡음이 독립이라는 EKF 가정이
깨져 공분산이 실제보다 작아지고, 결국 게이트가 정상 측정을 기각한다).

대신 이 노드가 image_topics에 나열된 모든 카메라를 구독하고, 카메라마다
자기 homography_files[i]로 world 좌표를 만든 다음, **역할(front/rear)별로
카메라 하나만 골라서** 발행한다. 선택 기준은 "그 카메라 광축 지상점에서
마커까지의 거리"가 가장 작은 카메라다 — parallax 오차가 광축 거리에
비례하므로 가장 정확한 관측을 쓰게 된다. camera_ground_*를 실측하지 않은
초기에는 마커 픽셀이 영상 중심에서 얼마나 떨어졌는지로 대체 판정한다.

선택된 카메라가 마커를 놓치면 즉시 다른 카메라로 넘어가되, 짧은 깜빡임으로
좌우로 튀지 않도록 hold 시간(selection_hold_s) 동안은 기존 카메라를 유지한다.

방식: 로봇 상판 마커는 CCTV가 거의 나달(수직 하방)로 내려다보므로
aruco_tracker_node처럼 카메라 intrinsics 기반 solvePnP가 필요 없다.
마커 4코너 픽셀좌표를 yolo_bev_map_node와 "같은" homography로 world
좌표로 변환하고, 코너 두 점(top-left→top-right, ArUco 표준 코너 순서라
마커가 이미지 안에서 어떻게 돌아가 있어도 항상 마커 자신의 로컬 +x변을
가리킨다) 사이의 world 벡터 방향을 로봇 yaw로 쓴다. 위치·자세를 같은
캘리브레이션 하나로 일관되게 얻는 게 목적.

알려진 한계 (실측 전 placeholder 취급할 것):
  1. 부착각 정렬: 마커의 "top-left→top-right" 변이 로봇 진행축(+x)과
     정확히 일치하게 부착해야 한다. 부착 오차는 yaw_offset_deg
     파라미터로 상쇄한다 — 실측 없이는 0으로 둬도 동작은 하지만
     yaw에 고정 바이어스가 남는다.
  2. Parallax(시차): yolo_bev_map_node의 homography는 "바닥" 평면
     기준으로 캘리브레이션됐다. 로봇 상판 마커는 바닥보다 높은 곳
     (로봇 섀시 높이만큼)에 있으므로, 카메라 광축에서 먼 위치일수록
     실제 위치보다 광축 반대방향으로 약간 밀려 보이는 오차가 생긴다.
     오차 크기 ≈ (마커 높이 / 카메라 설치 높이) × (광축에서 마커까지의
     수평거리) — 예: 카메라 2.5m, 마커 높이 0.12m, 광축에서 2m 떨어진
     위치라면 약 (0.12/2.5)×2m ≈ 9.6cm 오차. 나달 카메라 근사 보정은
     구현돼 있으나 camera/marker 높이와 광축의 바닥 교점 실측값이 필요하다.
     높이 파라미터가 0이면 보정은 의도적으로 비활성화된다. yaw는 마커 두
     코너가 서로 가까워 시차 영향이 위치보다 작다.
  3. 렌즈 왜곡: upstream cctv_rectify_node가 보정한다. 이 노드와
     yolo_bev_map_node는 반드시 동일한 /cctv/image_rect를 사용해야 한다.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import os

from cooperative_parking_robot.aruco_utils import (
    ArucoDetectorCompat,
    marker_center_to_base_link,
)
from cooperative_parking_robot.vision_utils import (
    correct_floor_projection,
    select_marker_by_id,
)

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    DEPS_OK = True
except ImportError:
    DEPS_OK = False


class CctvRobotMarkerNode(Node):
    def __init__(self):
        super().__init__('cctv_robot_marker_node')

        # ===== 파라미터 =====
        self.declare_parameter('image_topic', '/cctv/image_rect')
        self.declare_parameter('zero_stamp_fallback_to_now', True)
        self.declare_parameter('homography_file', 'homography_rectified.npy')
        # --- v1.11 멀티 카메라 ---
        # image_topics가 비어 있으면 기존 단일 image_topic/homography_file을
        # 그대로 쓴다(하위호환). 채워 넣으면 homography_files와 camera_ids도
        # 같은 길이여야 하며, 인덱스가 서로 짝을 이룬다.
        self.declare_parameter('image_topics', [''])
        self.declare_parameter('homography_files', [''])
        self.declare_parameter('camera_ids', [''])
        # launch에서 PathJoinSubstitution으로 만든 경로는 "문자열 하나"로만
        # 전달되므로, 배열 파라미터에 넣으면 substitution이 하나로 붙어버린다.
        # 그래서 launch용으로는 쉼표 구분 문자열 형태를 별도로 받는다.
        # (직접 YAML을 쓸 때는 위의 배열 파라미터가 더 읽기 좋다.)
        self.declare_parameter('image_topics_csv', '')
        self.declare_parameter('homography_files_csv', '')
        self.declare_parameter('camera_ids_csv', '')
        # 카메라별 광축 지상점 [x0,y0, x2,y2, ...] — 비우면 영상 중심 거리로 대체
        self.declare_parameter('camera_ground_points', [0.0])
        self.declare_parameter('camera_heights_m', [0.0])
        # 더 좋은 카메라로 넘어가기 전에 현재 카메라를 유지하는 최소 시간.
        self.declare_parameter('selection_hold_s', 0.30)
        # 이 시간이 지난 관측은 카메라 선택 후보에서 제외한다.
        self.declare_parameter('observation_timeout_s', 0.30)
        # BEV 브라우저 등록 도구의 H는 픽셀->metre를 직접 출력한다.
        self.declare_parameter('homography_scale_to_m', 1.0)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('front_marker_id', 2)
        self.declare_parameter('rear_marker_id', 1)
        # 현장 640x480 영상에서 정상 17.5cm 마커는 약 11~30px/변이다.
        # 고정값과 프레임 면적 비율 중 큰 값을 써 해상도가 바뀌어도 같은
        # 각크기 기준을 유지한다(640x480에서는 100px²).
        self.declare_parameter('min_marker_area_px', 100.0)
        self.declare_parameter('min_marker_area_ratio', 0.0003)
        # 부착각 실측 오차 보정 — 실측 전엔 0.0 (§calibration 참조)
        self.declare_parameter('front_yaw_offset_deg', 0.0)
        self.declare_parameter('rear_yaw_offset_deg', 0.0)
        # 마커 바깥끝 부착 오프셋 [m] — 마커 중심이 로봇 회전중심(base_link)에서
        # 로봇 진행축(+x)으로 떨어진 거리. Front는 앞끝(+), Rear는 뒤끝(−).
        # 0.0이면 마커가 로봇 중심에 있다는 기존 가정과 동일(하위호환).
        # 실측값(마커 중심↔회전중심)을 넣기 전엔 0.0 placeholder.
        self.declare_parameter('front_marker_offset_x_m', 0.0)
        self.declare_parameter('rear_marker_offset_x_m', 0.0)
        # 바닥 homography를 상판 높이로 환산하는 선택적 parallax 보정.
        # camera_ground_*는 카메라 광축이 바닥과 만나는 map 좌표다.
        self.declare_parameter('camera_ground_x_m', 0.0)
        self.declare_parameter('camera_ground_y_m', 0.0)
        self.declare_parameter('camera_height_m', 0.0)
        self.declare_parameter('front_marker_height_m', 0.0)
        self.declare_parameter('rear_marker_height_m', 0.0)

        self.homography_scale_to_m = float(
            self.get_parameter('homography_scale_to_m').value)
        if self.homography_scale_to_m <= 0.0:
            raise ValueError('homography_scale_to_m must be positive')
        self.min_marker_area_px = float(
            self.get_parameter('min_marker_area_px').value)
        self.min_marker_area_ratio = float(
            self.get_parameter('min_marker_area_ratio').value)
        if self.min_marker_area_px < 0.0 or self.min_marker_area_ratio < 0.0:
            raise ValueError('marker area thresholds must be non-negative')

        self.marker_ids = {
            'front': self.get_parameter('front_marker_id').value,
            'rear': self.get_parameter('rear_marker_id').value,
        }
        self.yaw_offset = {
            'front': math.radians(self.get_parameter('front_yaw_offset_deg').value),
            'rear': math.radians(self.get_parameter('rear_yaw_offset_deg').value),
        }
        self.marker_offset_x = {
            'front': float(self.get_parameter('front_marker_offset_x_m').value),
            'rear': float(self.get_parameter('rear_marker_offset_x_m').value),
        }
        if 0 in self.marker_ids.values():
            raise ValueError('상판 marker ID는 Rear 카메라용 ID0과 달라야 함')
        if len(set(self.marker_ids.values())) != len(self.marker_ids):
            raise ValueError('Front/Rear 상판 marker ID는 서로 달라야 함')

        self.camera_ground = (
            self.get_parameter('camera_ground_x_m').value,
            self.get_parameter('camera_ground_y_m').value,
        )
        self.camera_height = self.get_parameter('camera_height_m').value
        self.marker_height = {
            'front': self.get_parameter('front_marker_height_m').value,
            'rear': self.get_parameter('rear_marker_height_m').value,
        }
        if any(h < 0.0 for h in self.marker_height.values()):
            raise ValueError('marker height must be non-negative')
        if not any(h > 0.0 for h in self.marker_height.values()):
            self.get_logger().warn(
                '상판 마커 높이가 0 — parallax 보정 비활성화(실측 필요)')

        # ===== 리소스 로드 =====
        if not DEPS_OK:
            raise RuntimeError(
                'CCTV ArUco dependencies missing (cv2, numpy, cv_bridge)')
        self.bridge = CvBridge()
        self.selection_hold_s = float(
            self.get_parameter('selection_hold_s').value)
        self.observation_timeout_s = float(
            self.get_parameter('observation_timeout_s').value)
        if self.selection_hold_s < 0.0 or self.observation_timeout_s <= 0.0:
            raise ValueError(
                'selection_hold_s must be >= 0 and observation_timeout_s > 0')
        self._configure_cameras()
        self._setup_aruco()

        # ===== 구독 (카메라마다 하나) =====
        for camera in self.cameras:
            self.create_subscription(
                Image, camera['image_topic'],
                lambda msg, c=camera['camera_id']: self.image_cb(c, msg),
                qos_profile_sensor_data)

        # ===== 발행 (역할별, 카메라 수와 무관하게 하나씩) =====
        self.pub_pose = {
            role: self.create_publisher(
                PoseStamped, f'/{role}/cctv_pose', qos_profile_sensor_data)
            for role in self.marker_ids
        }
        self.pub_visible = {
            role: self.create_publisher(
                Bool, f'/{role}/cctv_marker_visible',
                qos_profile_sensor_data)
            for role in self.marker_ids
        }

        self._last_visible = {role: False for role in self.marker_ids}
        # role -> {camera_id: {'pose':(x,y,yaw), 'cost':float, 'wall':float}}
        self._observations = {role: {} for role in self.marker_ids}
        # role -> (camera_id, 선택된 시각)
        self._selected = {role: (None, 0.0) for role in self.marker_ids}
        self.get_logger().info(
            f"cctv_robot_marker_node 시작 (markers={self.marker_ids}, "
            f"cameras={[c['camera_id'] for c in self.cameras]}, "
            f"min_area={self.min_marker_area_px:.0f}px)")

    # ------------------------------------------------------------------
    # 카메라 구성
    # ------------------------------------------------------------------
    def _configure_cameras(self):
        """단일/멀티 카메라 파라미터를 하나의 cameras 리스트로 정규화한다."""
        def _csv(name):
            raw = str(self.get_parameter(name).value or '')
            return [part.strip() for part in raw.split(',') if part.strip()]

        def _array(name):
            return [str(v).strip()
                    for v in self.get_parameter(name).value
                    if str(v).strip()]

        # CSV 형태가 채워져 있으면 그쪽이 우선한다(launch 경로).
        topics = _csv('image_topics_csv') or _array('image_topics')
        files = _csv('homography_files_csv') or _array('homography_files')
        ids = _csv('camera_ids_csv') or _array('camera_ids')
        ground = [float(v)
                  for v in self.get_parameter('camera_ground_points').value]
        heights = [float(v)
                   for v in self.get_parameter('camera_heights_m').value]

        if not topics:
            # 하위호환 경로: 기존 단일 카메라 파라미터를 그대로 쓴다.
            topics = [str(self.get_parameter('image_topic').value).strip()]
            files = [str(self.get_parameter('homography_file').value).strip()]
            ids = ['cctv0']
            ground = [float(self.camera_ground[0]), float(self.camera_ground[1])]
            heights = [float(self.camera_height)]
            if not topics[0]:
                raise ValueError('image_topic must not be empty')

        if len(files) != len(topics):
            raise ValueError(
                'homography_files must have the same length as image_topics')
        if not ids:
            ids = [f'cctv{index}' for index in range(len(topics))]
        if len(ids) != len(topics):
            raise ValueError(
                'camera_ids must have the same length as image_topics')
        if len(set(ids)) != len(ids):
            raise ValueError('camera_ids must be unique')
        if len(set(topics)) != len(topics):
            raise ValueError('image_topics must be unique')
        # 광축 지상점은 카메라당 2개(x,y). 길이가 맞지 않으면 사용하지 않는다.
        if len(ground) != 2 * len(topics):
            if len(ground) > 1:
                self.get_logger().warn(
                    'camera_ground_points 길이가 카메라 수와 맞지 않아 무시 — '
                    '영상 중심 거리로 카메라를 선택합니다')
            ground = []
        if len(heights) != len(topics):
            if len(heights) == 1 and heights[0] == 0.0:
                heights = [float(self.camera_height)] * len(topics)
            else:
                raise ValueError(
                    'camera_heights_m must have the same length as image_topics')
        if any(not math.isfinite(height) or height < 0.0 for height in heights):
            raise ValueError('camera heights must be finite and non-negative')
        maximum_marker_height = max(self.marker_height.values())
        if maximum_marker_height > 0.0 and any(
                height <= maximum_marker_height for height in heights):
            raise ValueError(
                'every camera height must be greater than marker heights')

        self.cameras = []
        for index, topic in enumerate(topics):
            homography = self._load_homography(files[index], ids[index])
            axis = None
            if ground:
                axis_x, axis_y = ground[2 * index], ground[2 * index + 1]
                if axis_x != 0.0 or axis_y != 0.0:
                    axis = (axis_x, axis_y)
            self.cameras.append({
                'camera_id': ids[index],
                'image_topic': topic,
                'homography': homography,
                'axis_ground': axis,
                'height_m': heights[index],
            })
        self.camera_by_id = {
            camera['camera_id']: camera for camera in self.cameras}
        # 기존 단일 카메라 코드/테스트가 참조하는 대표 토픽 속성을 유지한다.
        self.image_topic = self.cameras[0]['image_topic']
        if not any(camera['axis_ground'] for camera in self.cameras):
            self.get_logger().warn(
                'camera_ground_points 미실측 — 마커가 영상 중심에서 얼마나 '
                '떨어졌는지로 카메라를 선택합니다(정확도는 충분하나 실측 권장)')

    def _load_homography(self, path, camera_id):
        """카메라 하나의 H를 읽는다. 없으면 즉시 실패시킨다."""
        expanded = os.path.expanduser(str(path))
        if not os.path.exists(expanded):
            raise FileNotFoundError(
                f'[{camera_id}] {expanded} 없음 — '
                '잘못된 절대 pose 발행을 막기 위해 시작 중단')
        matrix = np.load(expanded)
        if matrix.shape != (3, 3):
            raise ValueError(f'[{camera_id}] homography matrix must be 3x3')
        if not np.all(np.isfinite(matrix)):
            raise ValueError(
                f'[{camera_id}] homography matrix contains non-finite values')
        if abs(float(np.linalg.det(matrix))) < 1e-12:
            raise ValueError(f'[{camera_id}] homography matrix is singular')
        self.get_logger().info(f'[{camera_id}] 호모그래피 로드: {expanded}')
        return matrix

    def _setup_aruco(self):
        dict_name = self.get_parameter('aruco_dict').value
        self.detector = ArucoDetectorCompat(cv2, dict_name)

    def pixel_to_world(self, homography, px, py):
        """yolo_bev_map_node.pixel_to_world와 동일 관례 —
        출력 단위는 homography_scale_to_m 파라미터로 m에 맞춘다."""
        if homography is None:
            return None
        pt = np.array([px, py, 1.0])
        r = homography @ pt
        if abs(r[2]) < 1e-10:
            return None
        return (
            r[0] / r[2] * self.homography_scale_to_m,
            r[1] / r[2] * self.homography_scale_to_m,
        )

    # ================================================
    # 이미지 콜백
    # ================================================
    def image_cb(self, camera_id, msg):
        camera = self.camera_by_id[camera_id]
        image_stamp = msg.header.stamp
        if image_stamp.sec == 0 and image_stamp.nanosec == 0:
            if self.get_parameter('zero_stamp_fallback_to_now').value:
                image_stamp = self.get_clock().now().to_msg()
                self.get_logger().warn(
                    f'[{camera_id}] CCTV Image timestamp가 0 — 수신시각으로 대체',
                    throttle_duration_sec=5.0)
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detect_markers(gray)

        frame_height, frame_width = frame.shape[:2]
        now = time.monotonic()
        for role, marker_id in self.marker_ids.items():
            px_corners, _marker_area = select_marker_by_id(
                corners,
                ids,
                marker_id,
                min_area_px=self.min_marker_area_px,
                min_area_ratio=self.min_marker_area_ratio,
                frame_width=frame_width,
                frame_height=frame_height,
            )
            if px_corners is None:
                # 이 카메라는 이번 프레임에 못 봤다. 관측을 지워야 다른
                # 카메라가 선택될 수 있다.
                self._observations[role].pop(camera_id, None)
                continue
            pose = self._corners_to_pose(camera, px_corners, role)
            if pose is None:
                self._observations[role].pop(camera_id, None)
                continue
            self._observations[role][camera_id] = {
                'pose': pose,
                'cost': self._selection_cost(
                    camera, pose, px_corners, frame_width, frame_height),
                'wall': now,
                'stamp': image_stamp,
            }

        self._publish_selected(now)

    def _selection_cost(self, camera, pose, px_corners, frame_width,
                        frame_height):
        """작을수록 좋은 관측. parallax 오차의 대리 지표다.

        광축 지상점을 실측했으면 world 거리를, 아니면 마커 픽셀 중심이 영상
        중심에서 얼마나 떨어졌는지를 정규화해서 쓴다. 어차피 카메라 간
        상대 비교만 하므로 단위는 중요하지 않다.
        """
        axis = camera['axis_ground']
        if axis is not None:
            return math.hypot(pose[0] - axis[0], pose[1] - axis[1])
        center_x = sum(float(point[0]) for point in px_corners) / len(px_corners)
        center_y = sum(float(point[1]) for point in px_corners) / len(px_corners)
        return math.hypot(
            (center_x - frame_width / 2.0) / max(1.0, frame_width),
            (center_y - frame_height / 2.0) / max(1.0, frame_height))

    def _publish_selected(self, now):
        """역할별로 카메라 하나만 골라 /{role}/cctv_pose를 발행한다."""
        for role in self.marker_ids:
            fresh = {
                camera_id: observation
                for camera_id, observation in self._observations[role].items()
                if now - observation['wall'] <= self.observation_timeout_s
            }
            # 만료된 관측은 정리한다.
            self._observations[role] = fresh

            current_id, selected_at = self._selected[role]
            chosen_id = None
            if fresh:
                best_id = min(fresh, key=lambda c: fresh[c]['cost'])
                if (current_id in fresh and
                        now - selected_at < self.selection_hold_s):
                    # hold 구간 — 짧은 깜빡임으로 카메라가 튀지 않게 유지.
                    chosen_id = current_id
                else:
                    chosen_id = best_id
                    if chosen_id != current_id:
                        self._selected[role] = (chosen_id, now)
                        if current_id is not None:
                            self.get_logger().info(
                                f'[{role}] CCTV 전환 {current_id} -> {chosen_id}')
            else:
                self._selected[role] = (None, now)

            visible = chosen_id is not None
            message = Bool()
            message.data = visible
            self.pub_visible[role].publish(message)
            if visible:
                observation = fresh[chosen_id]
                x, y, yaw = observation['pose']
                out = PoseStamped()
                out.header.stamp = observation['stamp']
                out.header.frame_id = 'map'
                out.pose.position.x = x
                out.pose.position.y = y
                out.pose.orientation.z = math.sin(yaw / 2.0)
                out.pose.orientation.w = math.cos(yaw / 2.0)
                self.pub_pose[role].publish(out)
            if visible != self._last_visible[role]:
                suffix = f' ({chosen_id})' if visible else ''
                self.get_logger().info(
                    f"[{role}] CCTV 상판 마커 "
                    f"{'인식' if visible else '놓침'}{suffix}")
            self._last_visible[role] = visible

    def _corners_to_pose(self, camera, px_corners, role):
        """마커 4코너 픽셀 → 로봇 중심(base_link) world (x, y, yaw).
        yaw는 top-left→top-right 코너 벡터 방향 + 부착각 보정.
        위치는 마커 중심을 구한 뒤, 바깥끝 부착 오프셋을 로봇 진행축으로
        빼서 base_link 좌표로 환산한다 (marker_offset_x=0이면 마커 중심 그대로)."""
        homography = camera['homography']
        world_pts = []
        for px, py in px_corners:
            w = self.pixel_to_world(homography, float(px), float(py))
            if w is None:
                return None
            world_pts.append(self._correct_parallax(camera, w[0], w[1], role))
        cx = sum(p[0] for p in world_pts) / 4.0
        cy = sum(p[1] for p in world_pts) / 4.0
        tl, tr = world_pts[0], world_pts[1]
        yaw = math.atan2(tr[1] - tl[1], tr[0] - tl[0]) + self.yaw_offset[role]
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        # 마커 중심 → base_link 환산 (바깥끝 부착 보정)
        bx, by = marker_center_to_base_link(cx, cy, yaw, self.marker_offset_x[role])
        return bx, by, yaw

    def _correct_parallax(self, camera, floor_x, floor_y, role):
        '''Convert a floor-plane ray intersection to marker-height position.

        parallax 보정은 "그 카메라의" 광축 지상점 기준이어야 한다. 두 카메라가
        같은 map frame을 공유해도 광축 위치는 서로 다르기 때문이다.
        '''
        axis = camera['axis_ground'] or self.camera_ground
        return correct_floor_projection(
            floor_x, floor_y,
            axis[0], axis[1],
            camera['height_m'], self.marker_height[role])


def main(args=None):
    rclpy.init(args=args)
    node = CctvRobotMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
