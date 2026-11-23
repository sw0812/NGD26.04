#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO  

class CameraNode:
    def __init__(self):
        rospy.loginfo("Camera_read")
        self.bridge = CvBridge()
        self.model = YOLO("/home/sasllab/yolov8n.engine")

        self.image1 = None
        self.image2 = None

        # 퍼블리셔
        self.center_pub1 = rospy.Publisher("/camera1/object_center", Point, queue_size=10)
        self.center_pub2 = rospy.Publisher("/camera2/object_center", Point, queue_size=10)

        # 이미지 구독자
        self.image_sub1 = rospy.Subscriber("/camera1/image_raw", Image, self.callback_camera1)
        self.image_sub2 = rospy.Subscriber("/camera2/image_raw", Image, self.callback_camera2)

        # 디스플레이 및 YOLO 주기적 처리
        rospy.Timer(rospy.Duration(1.0 / 15.0), self.display_callback)  # 15Hz 정도 권장

    def callback_camera1(self, data):
        try:
            self.image1 = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("Camera1 error: {}".format(e))

    def callback_camera2(self, data):
        try:
            self.image2 = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("Camera2 error: {}".format(e))

    def display_callback(self, event):
        if self.image1 is not None:
            self.process_detections(self.image1, "Camera1 with YOLOv8", self.center_pub1)
        if self.image2 is not None:
            self.process_detections(self.image2, "Camera2 with YOLOv8", self.center_pub2)

        key = cv2.waitKey(1)
        if key == 27:  # ESC
            rospy.signal_shutdown("ESC pressed")

    def process_detections(self, cv_image, window_name, publisher):
        try:
            results = self.model(cv_image)
            annotated_frame = results[0].plot()

            if results[0].boxes is not None:
                boxes = results[0].boxes.xyxy.cpu().numpy()
                for box in boxes:
                    x1, y1, x2, y2 = box
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    # 중심점 표시
                    cv2.circle(annotated_frame, (center_x, center_y), 10, (0, 0, 255), -1)
                    rospy.loginfo(f"{window_name} - Object center: ({center_x}, {center_y})")

                    # 퍼블리시
                    center_msg = Point()
                    center_msg.x = center_x
                    center_msg.y = center_y
                    center_msg.z = 0.0
                    publisher.publish(center_msg)

            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            resized = cv2.resize(annotated_frame, (920, 720))
            cv2.imshow(window_name, resized)

        except Exception as e:
            rospy.logerr(f"[YOLO] Processing error in {window_name}: {e}")

def main():
    rospy.init_node('camera_read', anonymous=False)
    CameraNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


