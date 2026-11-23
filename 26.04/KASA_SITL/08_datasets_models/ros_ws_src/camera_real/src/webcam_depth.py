#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

class DepthEstimatorWebcam:
    def __init__(self):
        rospy.init_node('webcam_depth_estimator', anonymous=False)

        # 카메라 파라미터 설정
        self.image_width = 920
        self.image_height = 720

        self.fx = 1000.0  # focal length in pixels (실측값 or 대략값)
        self.fy = 1000.0
        self.cx = self.image_width / 2.0
        self.cy = self.image_height / 2.0
        self.baseline = 0.1  # 실제 웹캠 간 거리 [m], 예: 10cm

        self.center1 = None
        self.center2 = None

        # 웹캠 토픽 구독
        rospy.Subscriber("/webcam1/object_center", Point, self.callback_webcam1)
        rospy.Subscriber("/webcam2/object_center", Point, self.callback_webcam2)

        rospy.loginfo("DepthEstimator (webcam) node initialized")
        rospy.spin()

    def callback_webcam1(self, msg):
        self.center1 = (msg.x, msg.y)
        self.try_compute_depth()

    def callback_webcam2(self, msg):
        self.center2 = (msg.x, msg.y)
        self.try_compute_depth()

    def try_compute_depth(self):
        if self.center1 is not None and self.center2 is not None:
            disparity = abs(self.center1[0] - self.center2[0])  # disparity = x1 - x2

            if disparity > 0:
                z = (self.fx * self.baseline) / disparity

                x = (self.center1[0] - self.cx) * z / self.fx
                y = (self.center1[1] - self.cy) * z / self.fy

                rospy.loginfo("[3D 좌표] X: %.2f m, Y: %.2f m, Z: %.2f m", x, y, z)
            else:
                rospy.logwarn("Disparity is zero. Cannot compute depth.")

if __name__ == '__main__':
    try:
        DepthEstimatorWebcam()
    except rospy.ROSInterruptException:
        pass
