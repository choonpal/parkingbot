from setuptools import setup
import os
from glob import glob

package_name = 'cooperative_parking_robot'

setup(
    name=package_name,
    version='1.11.3',
    packages=[package_name],
    package_data={package_name: ['web/*.html', 'web/*.css', 'web/*.js']},
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'opencv_camera = cooperative_parking_robot.opencv_camera_node:main',
            'cctv_rectify = cooperative_parking_robot.cctv_rectify_node:main',
            'jetson_vision_web = cooperative_parking_robot.jetson_vision_web_node:main',
            ('bev_layout_calibrator = '
             'cooperative_parking_robot.mvp_integration_nodes:'
             'bev_layout_calibrator_main'),
            ('yolo_bev_map = '
             'cooperative_parking_robot.yolo_bev_map_production_node:main'),
            ('yolo_bev_map_baseline = '
             'cooperative_parking_robot.mvp_integration_nodes:yolo_bev_map_main'),
            ('fleet_manager = '
             'cooperative_parking_robot.mvp_fleet_manager_node:main'),
            'ultrasonic_edge = cooperative_parking_robot.ultrasonic_edge_node:main',
            'aruco_tracker = cooperative_parking_robot.aruco_tracker_node:main',
            ('cctv_robot_marker = '
             'cooperative_parking_robot.cctv_robot_marker_production_node:main'),
            ('cctv_robot_marker_baseline = '
             'cooperative_parking_robot.cctv_robot_marker_node:main'),
            'cctv_merge = cooperative_parking_robot.cctv_merge_global_vehicle_node:main',
            # Production keeps the existing command-owner/completion-first
            # stack, and replaces only ALIGN_SLOT_YAW with Q/E phase control.
            ('rigid_body_sync = '
             'cooperative_parking_robot.rigid_body_sync_vehicle_global_node:main'),
            ('rigid_body_sync_mvp_baseline = '
             'cooperative_parking_robot.mvp_runtime_nodes:'
             'rigid_body_sync_main'),
            ('rigid_body_sync_p0_baseline = '
             'cooperative_parking_robot.rigid_body_sync_production_node:main'),
            ('rigid_body_sync_safe_baseline = '
             'cooperative_parking_robot.rigid_body_sync_safe_node:main'),
            ('rigid_body_sync_legacy = '
             'cooperative_parking_robot.rigid_body_sync_node:main'),
            ('state_machine = '
             'cooperative_parking_robot.mvp_state_machine_node:main'),
            ('stm32_bridge = '
             'cooperative_parking_robot.mvp_stm32_bridge_node:main'),
            ('individual_move = '
             'cooperative_parking_robot.mvp_runtime_nodes:'
             'individual_move_main'),
            ('pose_fusion = '
             'cooperative_parking_robot.pose_fusion_production_node:main'),
            ('pose_fusion_baseline = '
             'cooperative_parking_robot.pose_fusion_node:main'),
            'hardware_preflight = cooperative_parking_robot.hardware_preflight:main',
            ('drive_pulse = '
             'cooperative_parking_robot.drive_pulse_node:main'),
            'cooperative_drive_test = cooperative_parking_robot.cooperative_drive_test_node:main',
            'calibrate_camera = cooperative_parking_robot.calibrate_camera_node:main',
            'camera_preview = cooperative_parking_robot.camera_preview_node:main',
            'show_map_ascii = cooperative_parking_robot.show_map_ascii:main',
            'tile_homography = cooperative_parking_robot.tile_homography_node:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.npz')),
        (os.path.join('share', package_name, 'models'),
         glob('models/*.pt')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
)
