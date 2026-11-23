#!/usr/bin/env python3
"""Draws aruco_tracker's detections onto the camera image and republishes it.

aruco_opencv's ArucoDetection message only carries the solved 3D pose per
marker (no pixel corners), so the overlay is built by re-projecting each
marker's pose (and a small square the size of the physical marker) back into
the image with the camera intrinsics from CameraInfo.
"""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from aruco_opencv_msgs.msg import ArucoDetection


def quaternion_to_rvec(q):
    """geometry_msgs/Quaternion -> cv2 Rodrigues rotation vector."""
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    rvec, _ = cv2.Rodrigues(R)
    return rvec


class ArucoDebugView(Node):

    def __init__(self):
        super().__init__('aruco_debug_view')

        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('detections_topic', '/aruco_detections')
        self.declare_parameter('debug_image_topic', '/aruco_debug_image')
        self.declare_parameter('marker_size', 0.5)  # m, must match aruco_tracker.yaml

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        debug_image_topic = self.get_parameter('debug_image_topic').value
        self.marker_size = float(self.get_parameter('marker_size').value)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_detection = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(CameraInfo, camera_info_topic, self.on_camera_info, sensor_qos)
        self.create_subscription(ArucoDetection, detections_topic, self.on_detection, 10)
        self.create_subscription(Image, image_topic, self.on_image, sensor_qos)
        self.debug_pub = self.create_publisher(Image, debug_image_topic, 10)

        self.get_logger().info(
            f'image={image_topic} camera_info={camera_info_topic} '
            f'detections={detections_topic} -> {debug_image_topic}'
        )

    def on_camera_info(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64) if msg.d else np.zeros(5)

    def on_detection(self, msg: ArucoDetection):
        self.latest_detection = msg

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.camera_matrix is None:
            self.publish(frame, msg.header)
            return

        det = self.latest_detection
        stale = det is None or (self.get_clock().now().nanoseconds -
                                 (det.header.stamp.sec * 1_000_000_000 + det.header.stamp.nanosec)) > 5e8

        if det is None or not det.markers:
            cv2.putText(frame, 'NO MARKER', (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, (0, 0, 255), 2)
        else:
            half = self.marker_size / 2.0
            square = np.array([
                [-half, half, 0],
                [half, half, 0],
                [half, -half, 0],
                [-half, -half, 0],
            ], dtype=np.float64)

            for marker in det.markers:
                p = marker.pose.position
                tvec = np.array([[p.x], [p.y], [p.z]], dtype=np.float64)
                rvec = quaternion_to_rvec(marker.pose.orientation)

                corners_2d, _ = cv2.projectPoints(
                    square, rvec, tvec, self.camera_matrix, self.dist_coeffs)
                corners_2d = corners_2d.reshape(-1, 2).astype(int)
                cv2.polylines(frame, [corners_2d], True, (0, 255, 0), 2)

                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                                   rvec, tvec, half)

                dist = math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2)
                label_pos = tuple(corners_2d[0])
                cv2.putText(frame, f'id={marker.marker_id} d={dist:.2f}m',
                            (label_pos[0], max(label_pos[1] - 10, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if stale:
                cv2.putText(frame, 'DETECTION STALE (>0.5s)', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 165, 255), 2)

        self.publish(frame, msg.header)

    def publish(self, frame, header):
        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = header
        self.debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDebugView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
