#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO

class WebcamYOLO:
    def __init__(self):
        rospy.init_node('webcam_yolo_node')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n_uav.pt')  # 경로 수정

        # 퍼블리셔
        self.center_pub1 = rospy.Publisher("/webcam1/object_center", Point, queue_size=10)
        self.center_pub2 = rospy.Publisher("/webcam2/object_center", Point, queue_size=10)

        # 웹캠 열기
        self.cap1 = cv2.VideoCapture(0)
        self.cap2 = cv2.VideoCapture(2)  # 두 번째 카메라가 있을 경우

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            ret1, frame1 = self.cap1.read()
            ret2, frame2 = self.cap2.read()

            if ret1:
                self.process_detections(frame1, "Webcam 1", self.center_pub1)

            if ret2:
                self.process_detections(frame2, "Webcam 2", self.center_pub2)

            if not ret1 and not ret2:
                rospy.logwarn("웹캠에서 영상을 가져올 수 없습니다.")

            rate.sleep()

        self.cap1.release()
        self.cap2.release()
        cv2.destroyAllWindows()

    def process_detections(self, frame, window_name, publisher):
        results = self.model(frame)
        annotated_frame = results[0].plot()

        if results[0].boxes is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            for box in boxes:
                x1, y1, x2, y2 = box
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                cv2.circle(annotated_frame, (cx, cy), 5, (0, 0, 255), -1)

                # 퍼블리시
                point = Point()
                point.x = cx
                point.y = cy
                point.z = 0.0
                publisher.publish(point)

                rospy.loginfo(f"{window_name} object center: ({cx}, {cy})")

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, cv2.resize(annotated_frame, (920, 720)))
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        node = WebcamYOLO()
        node.run()
    except rospy.ROSInterruptException:
        pass
