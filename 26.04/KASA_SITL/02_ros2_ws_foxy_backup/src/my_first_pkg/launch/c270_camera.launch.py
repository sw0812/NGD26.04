"""C270 웹캠을 usb_cam_node_exe로 띄워서 search_phase_node.py /
aruco_tracker가 기대하는 /camera/image_raw, /camera/camera_info로 퍼블리시.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_path = os.path.join(
        get_package_share_directory('my_first_pkg'),
        'config',
        'c270_usb_cam.yaml',
    )

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            namespace='camera',
            parameters=[params_path],
            output='screen',
        ),
    ])
