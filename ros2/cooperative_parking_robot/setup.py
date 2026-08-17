from setuptools import setup
import os
from glob import glob

package_name = 'cooperative_parking_robot'

setup(
    name=package_name,
    version='1.11.0',
    packages=[package_name],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'opencv_camera = cooperative_parking_robot.opencv_camera_node:main',
            'cctv_rectify = cooperative_parking_robot.cctv_rectify_node:main',
            'jetson_vision_web = cooperative_parking_robot.jetson_vision_web_node:main',
            'bev_layout_calibrator = cooperative_parking_robot.bev_layout_calibrator_node:main',
            'yolo_bev_map = cooperative_parking_robot.yolo_bev_map_node:main',
            'fleet_manager = cooperative_parking_robot.fleet_manager_node:main',
            'ultrasonic_edge = cooperative_parking_robot.ultrasonic_edge_node:main',
            'aruco_tracker = cooperative_parking_robot.aruco_tracker_node:main',
            'cctv_robot_marker = cooperative_parking_robot.cctv_robot_marker_node:main',
            'cctv_merge = cooperative_parking_robot.cctv_merge_node:main',
            'rigid_body_sync = cooperative_parking_robot.rigid_body_sync_node:main',
            'state_machine = cooperative_parking_robot.robot_state_machine_node:main',
            'stm32_bridge = cooperative_parking_robot.stm32_bridge_node:main',
            'individual_move = cooperative_parking_robot.individual_move_node:main',
            'pose_fusion = cooperative_parking_robot.pose_fusion_node:main',
            'hardware_preflight = cooperative_parking_robot.hardware_preflight:main',
            'calibrate_camera = cooperative_parking_robot.calibrate_camera_node:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.npz')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'stm32_firmware', 'Core', 'Src'),
         glob('stm32_firmware/Core/Src/*.c')),
    ],
)
