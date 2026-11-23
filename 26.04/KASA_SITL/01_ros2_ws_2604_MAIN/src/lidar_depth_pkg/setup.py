from setuptools import setup
import os
from glob import glob

package_name = 'lidar_depth_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sasllab',
    maintainer_email='sasllab@example.com',
    description='LiDAR depth estimation and precision landing dataset pipeline',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_map_node = lidar_depth_pkg.depth_map_node:main',
            'depth_view_node = lidar_depth_pkg.depth_view_node:main',
            'landing_spot_node = lidar_depth_pkg.landing_spot_node:main',
            'dataset_collector_node = lidar_depth_pkg.dataset_collector_node:main',
            'dataset_collector_node_v2 = lidar_depth_pkg.dataset_collector_node_v2:main',
            'yolo_inference_node = lidar_depth_pkg.yolo_inference_node:main',
            'landing_control_node = lidar_depth_pkg.landing_control_node:main',
            'path_planner_node = lidar_depth_pkg.path_planner_node:main',
            'obstacle_repulsion_node = lidar_depth_pkg.obstacle_repulsion_node:main',
            'lidar_calib_check = lidar_depth_pkg.lidar_calib_check:main',
        ],
    },
)
