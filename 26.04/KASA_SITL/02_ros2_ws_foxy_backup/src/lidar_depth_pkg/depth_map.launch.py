from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_depth_pkg',
            executable='depth_map_node',
            name='depth_map_node',
            output='screen',
            parameters=[{
                'input_topic': '/lidar/points',
                'output_topic': '/depth/image',
                'horizontal_samples': 360,
                'vertical_samples': 64,
                'vertical_min_deg': -50.0,
                'vertical_max_deg': 50.0,
                'range_min': 0.3,
                'range_max': 50.0,
            }],
        ),
        Node(
            package='lidar_depth_pkg',
            executable='depth_view_node',
            name='depth_view_node',
            output='screen',
            parameters=[{
                'input_topic': '/depth/image',
                'output_topic': '/depth/image_view',
                'vertical_upscale': 5,
            }],
        ),
        Node(
            package='lidar_depth_pkg',
            executable='landing_spot_node',
            name='landing_spot_node',
            output='screen',
            parameters=[{
                'input_topic': '/depth/image',
                'output_topic': '/landing/candidate_point',
                'horizontal_window': 30,
                'min_valid_fraction': 0.6,
                'range_max_for_landing': 10.0,
                'step': 5,
            }],
        ),
    ])
