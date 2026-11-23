#!/usr/bin/env python3
"""SITL/실기체 공용 라인트레이싱 시각화 노드.

/camera/image_raw 를 받아 search_phase_node.py와 같은 LineTracker를 돌리고,
test_line_tracker_camera.py의 draw_overlay(ROI, 검출점, e_y/e_psi, 이진화 마스크)를
그대로 얹어 /line_tracker/debug 로 발행한다. rqt_image_view로 보면 됨.

사용법:
    python3 line_tracker_viz_node.py
    ros2 run rqt_image_view rqt_image_view /line_tracker/debug
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

from line_tracker import LineTracker
from test_line_tracker_camera import draw_overlay


class LineTrackerViz(Node):
    def __init__(self):
        super().__init__("line_tracker_viz")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.bridge = CvBridge()
        self.tracker = LineTracker()
        self.pub = self.create_publisher(Image, "/line_tracker/debug", 1)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h = frame.shape[0]
        t = self.tracker
        y0, y1 = int(h * t.roi_top_ratio), int(h * t.roi_bottom_ratio)
        gray = cv2.GaussianBlur(cv2.cvtColor(frame[y0:y1, :], cv2.COLOR_BGR2GRAY), (5, 5), 0)
        mask = t._binarize(gray)
        result = t.process(frame)
        out = self.bridge.cv2_to_imgmsg(draw_overlay(frame, t, result, mask), encoding="bgr8")
        out.header = msg.header
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(LineTrackerViz())


if __name__ == "__main__":
    main()
