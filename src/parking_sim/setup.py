from setuptools import setup
from glob import glob
import os
setup(
    name='parking_sim',
    version='1.0.0',
    packages=['parking_sim'],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'robot = parking_sim.sim_robot:main',
        'cctv = parking_sim.sim_cctv:main',
        'controller = parking_sim.nav_controller:main',
        'visualizer = parking_sim.visualizer:main',
        'sim_aruco_sync = parking_sim.sim_aruco_sync:main',
    ]},
    data_files=[
        ('share/parking_sim', ['package.xml']),
        (os.path.join('share','parking_sim','launch'), glob('launch/*.py')),
    ],
)
