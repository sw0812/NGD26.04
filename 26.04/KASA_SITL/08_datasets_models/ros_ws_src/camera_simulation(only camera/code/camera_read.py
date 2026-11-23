#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO  

class CameraNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        # 토픽을 구독
        self.image_sub1 = rospy.Subscriber("/camera1/image_raw", Image, self.callback_camera1)
        self.image_sub2 = rospy.Subscriber("/camera2/image_raw", Image, self.callback_camera2)

    def process_detections(self, cv_image, window_name):
        # YOLOv8 
        results = self.model(cv_image)
        # 결과 이미지 시각화를 위한 기본 
        annotated_frame = results[0].plot()

        # 정보 추출 
        if results[0].boxes is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()  # [x1, y1, x2, y2]
            for box in boxes:
                x1, y1, x2, y2 = box
                width = x2 - x1   # 객체의 너비
                height = y2 - y1  # 객체의 높이
                center_x = int(x1 + width / 2)
                center_y = int(y1 + height / 2)
                # 중심 좌표
                cv2.circle(annotated_frame, (center_x, center_y), 10, (0, 0, 255), -1)
                # 좌표 출력
                rospy.loginfo(f"{window_name} - Detected object center: [x1: {x1:.1f}, y1: {y1:.1f}, x2: {x2:.1f}, y2: {y2:.1f}],({center_x}, {center_y})")
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # 윈도우 생성
        cv2.setMouseCallback(window_name, self.mouse_callback)

        resized_image = cv2.resize(annotated_frame, (920, 720))
        cv2.imshow(window_name, resized_image)
        cv2.waitKey(1)
    

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            rospy.loginfo(f"Mouse clicked at: ({x}, {y})")

    def callback_camera1(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("Camera1: {}".format(e))
            return
        self.process_detections(cv_image, "Camera1 with YOLOv8")

    def callback_camera2(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("Camera2: {}".format(e))
            return
        self.process_detections(cv_image, "Camera2 with YOLOv8")

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
