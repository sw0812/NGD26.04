"""LiDAR depth -> Hailo YOLO(landing_pad) -> /landing_target. (기존 lidar_depth_pkg 의 depth_map_node 필요)
ros2 launch kasa_hailo hailo_landing.launch.py hef_path:=/home/pi/landing_pad.hef"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hef = LaunchConfiguration('hef_path')
    return LaunchDescription([
        DeclareLaunchArgument('hef_path', default_value='/home/pi/landing_pad.hef'),
        DeclareLaunchArgument('imgsz', default_value='640'),
        Node(package='lidar_depth_pkg', executable='depth_map_node', name='depth_map_node', output='screen',
             parameters=[{'input_topic': '/lidar/points', 'output_topic': '/depth/image',
                          'horizontal_samples': 360, 'vertical_samples': 64,
                          'vertical_min_deg': -50.0, 'vertical_max_deg': 50.0,
                          'range_min': 0.3, 'range_max': 50.0}]),
        Node(package='kasa_hailo', executable='hailo_yolo_inference_node', name='yolo_inference_node',
             output='screen', parameters=[{'hef_path': hef, 'imgsz': LaunchConfiguration('imgsz')}]),
    ])
