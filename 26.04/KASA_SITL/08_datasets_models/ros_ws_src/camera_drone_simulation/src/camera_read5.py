#!/usr/bin/env python3

import rospy
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

import sys
sys.path.append('/home/sasllab/yolov5')  # YOLOv5 경로 맞게 수정

from models.common import DetectMultiBackend
from utils.datasets import letterbox
from utils.general import non_max_suppression
from utils.torch_utils import select_device

# 🔧 scale_boxes 함수는 v6.1에 없으므로 직접 정의
def scale_boxes(img1_shape, boxes, img0_shape, ratio_pad=None):
    # Rescale boxes (xyxy) from img1_shape to img0_shape
    if ratio_pad is None:
        gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])
        pad = ((img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2)
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    boxes[:, [0, 2]] -= pad[0]
    boxes[:, [1, 3]] -= pad[1]
    boxes[:, :4] /= gain
    boxes[:, 0].clamp_(0, img0_shape[1])
    boxes[:, 1].clamp_(0, img0_shape[0])
    boxes[:, 2].clamp_(0, img0_shape[1])
    boxes[:, 3].clamp_(0, img0_shape[0])
    return boxes

class CameraNode:
    def __init__(self):
        rospy.loginfo("camera_read")
        self.bridge = CvBridge()
        self.device = select_device('')
        self.model = DetectMultiBackend('/home/sasllab/yolov5/yolov5n_backup.pt', device=self.device)
        self.model.model.float().eval()

        self.img_size = 640
        self.conf_thres = 0.25
        self.iou_thres = 0.45

        self.image1 = None
        self.image2 = None

        self.center_pub1 = rospy.Publisher("/camera1/object_center", Point, queue_size=10)
        self.center_pub2 = rospy.Publisher("/camera2/object_center", Point, queue_size=10)

        self.image_sub1 = rospy.Subscriber("/camera1/image_raw", Image, self.callback_camera1)
        self.image_sub2 = rospy.Subscriber("/camera2/image_raw", Image, self.callback_camera2)

        rospy.Timer(rospy.Duration(1.0 / 15.0), self.display_callback)

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
            self.process(self.image1, "Camera1", self.center_pub1)
        if self.image2 is not None:
            self.process(self.image2, "Camera2", self.center_pub2)
        if cv2.waitKey(1) == 27:
            rospy.signal_shutdown("ESC pressed")

    def process(self, img_bgr, window_name, publisher):
        try:
            img = letterbox(img_bgr, self.img_size, stride=32, auto=True)[0]
            img = img.transpose((2, 0, 1))[::-1]  # BGR to RGB, HWC to CHW
            img = np.ascontiguousarray(img)

            img_tensor = torch.from_numpy(img).to(self.device).float() / 255.0
            if img_tensor.ndimension() == 3:
                img_tensor = img_tensor.unsqueeze(0)

            pred = self.model(img_tensor, augment=False, visualize=False)
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres)[0]

            annotated = img_bgr.copy()
            if pred is not None and len(pred):
                pred[:, :4] = scale_boxes(img_tensor.shape[2:], pred[:, :4], img_bgr.shape).round()
                for *xyxy, conf, cls in pred:
                    x1, y1, x2, y2 = map(int, xyxy)
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    cv2.circle(annotated, (cx, cy), 8, (0, 0, 255), -1)
                    rospy.loginfo(f"{window_name} - Object center: ({cx}, {cy})")

                    center = Point()
                    center.x = cx
                    center.y = cy
                    center.z = 0.0
                    publisher.publish(center)

            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            resized = cv2.resize(annotated, (920, 720))
            cv2.imshow(window_name, resized)

        except Exception as e:
            rospy.logerr(f"[YOLOv5] Error in {window_name}: {e}")

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
