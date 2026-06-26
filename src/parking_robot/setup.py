from setuptools import setup
import os
from glob import glob

package_name = 'parking_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            # [파트1] CCTV 서버
            'yolo_bev_map = parking_robot.yolo_bev_map_node:main',
            'fleet_manager = parking_robot.fleet_manager_node:main',
            # [파트2] 로봇 두뇌 (라즈베리파이)
            'ultrasonic_edge = parking_robot.ultrasonic_edge_node:main',
            'aruco_tracker = parking_robot.aruco_tracker_node:main',
            'rigid_body_sync = parking_robot.rigid_body_sync_node:main',
            'state_machine = parking_robot.robot_state_machine_node:main',
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
)
