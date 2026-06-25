from setuptools import find_packages, setup

package_name = 'cctv_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shiha',
    maintainer_email='shihankyul@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cctv_fusion_node = cctv_perception.cctv_fusion_node:main'
            ,'calibrate=cctv_perception.calibrate:main',
        ],
    },
)
