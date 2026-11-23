"""LiDAR depth -> Hailo YOLO(landing_pad) -> /landing_target. (기존 lidar_depth_pkg 의 depth_map_node 필요)
ros2 launch kasa_hailo hailo_landing.launch.py   (호스트의 hailo_server 가 떠 있어야 함)"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('server_port', default_value='5555'),
        DeclareLaunchArgument('imgsz', default_value='640'),
        Node(package='lidar_depth_pkg', executable='depth_map_node', name='depth_map_node', output='screen',
             parameters=[{'input_topic': '/lidar/points', 'output_topic': '/depth/image',
                          'horizontal_samples': 360, 'vertical_samples': 64,
                          'vertical_min_deg': -50.0, 'vertical_max_deg': 50.0,
                          'range_min': 0.3, 'range_max': 50.0}]),
        Node(package='kasa_hailo', executable='hailo_yolo_inference_node', name='yolo_inference_node',
             output='screen', parameters=[{'server_port': LaunchConfiguration('server_port'), 'imgsz': LaunchConfiguration('imgsz')}]),
    ])
