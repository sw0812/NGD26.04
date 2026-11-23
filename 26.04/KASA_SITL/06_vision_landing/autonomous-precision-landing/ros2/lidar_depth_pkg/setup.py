import os
from glob import glob
from setuptools import setup

package_name = 'lidar_depth_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sasllab',
    maintainer_email='you@example.com',
    description='LiDAR PointCloud2 -> Depth Map 변환 노드 (LiDAR 정밀착륙 프로젝트)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_map_node = lidar_depth_pkg.depth_map_node:main',
            'depth_view_node = lidar_depth_pkg.depth_view_node:main',
            'landing_spot_node = lidar_depth_pkg.landing_spot_node:main',
            'dataset_collector_node = lidar_depth_pkg.dataset_collector_node:main',
        ],
    },
)
