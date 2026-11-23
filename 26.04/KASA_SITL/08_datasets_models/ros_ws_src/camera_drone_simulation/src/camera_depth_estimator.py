#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates
import numpy as np

class DepthEstimator:
    def __init__(self):
        rospy.init_node('camera_depth_estimator', anonymous=False)

        # 카메라 파라미터 설정
        self.image_width = 920
        self.image_height = 720

        self.fx = 1000  # focal length [단위:pixels]
        self.fy = 1000
        self.cx = self.image_width / 2
        self.cy = self.image_height / 2
        self.baseline = 3   # [단위:m]

        # 좌우 중심점 초기화
        self.center1 = None
        self.center2 = None

        # 드론 위치 초기화
        self.drone_pos = None

        # 중앙 카메라의 실제 위치 (camera1과 camera2 평균 위치)
        self.camera_pos = np.array([5.01, 0.0, 0.05])

        # 토픽 구독
        rospy.Subscriber("/camera1/object_center", Point, self.callback_camera1)
        rospy.Subscriber("/camera2/object_center", Point, self.callback_camera2)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.update_drone_position)

        rospy.loginfo("DepthEstimator node initialized")
        rospy.spin()

    def update_drone_position(self, msg):
        if "Kwad" in msg.name:
            idx = msg.name.index("Kwad")
            pose = msg.pose[idx].position
            self.drone_pos = np.array([pose.x, pose.y, pose.z])
            # 드론 위치는 추정 수행 시에만 출력되도록 수정됨

    def callback_camera1(self, msg):
        self.center1 = (msg.x, msg.y)
        self.try_compute_depth()

    def callback_camera2(self, msg):
        self.center2 = (msg.x, msg.y)
        self.try_compute_depth()

    def try_compute_depth(self):
        if self.center1 is not None and self.center2 is not None:
            disparity = abs(self.center1[0] - self.center2[0])  # x 좌표 차이 [절대값]

            if disparity > 0:
                depth = (self.fx * self.baseline) / disparity  # 실제 깊이 (전방 거리)

                # 중심 기준으로 정규화된 좌표를 계산
                u_avg = (self.center1[0] + self.center2[0]) / 2.0
                v_avg = (self.center1[1] + self.center2[1]) / 2.0

                # 좌표계 보정: depth → x, 가로 → y, 세로 → z
                x = depth
                y = (u_avg - self.cx) * depth / self.fx
                z = abs((v_avg - self.cy) * depth / self.fy)

                rospy.loginfo("[3D 좌표 - 중앙 기준] 깊이: %.2f m, 가로: %.2f m, 세로: %.2f m", x, y, z)

                # 절대 좌표 변환: 카메라 위치를 기준으로 추정 위치를 변환
                estimated_world = np.array([self.camera_pos[0] - x,  # 카메라의 x위치 - 깊이
                                            self.camera_pos[1] + y,  # 가로: 좌우는 동일 축 방향이므로 +
                                            self.camera_pos[2] + z]) # 높이

                if self.drone_pos is not None:
                    rospy.loginfo("[드론 실제 위치] X: %.3f, Y: %.3f, Z: %.3f", self.drone_pos[0], self.drone_pos[1], self.drone_pos[2])

                    error_vec = estimated_world - self.drone_pos
                    error_dist = np.linalg.norm(error_vec)

                    rospy.loginfo("[추정 vs 드론 위치 비교] 오차 거리: %.3f m, 오차 벡터: [%.3f, %.3f, %.3f]",
                                  error_dist, error_vec[0], error_vec[1], error_vec[2])

                    error_x = abs(error_vec[0])
                    error_y = abs(error_vec[1])
                    error_z = abs(error_vec[2])
                    rospy.loginfo("[축별 오차] 깊이 오차: %.3f m, 좌우 오차: %.3f m, 상하 오차: %.3f m",
                                  error_x, error_y, error_z)
                else:
                    rospy.logwarn("드론 위치 정보를 아직 받지 못했습니다.")
            else:
                rospy.logwarn("Disparity is zero. Cannot compute depth.")

if __name__ == '__main__':
    try:
        DepthEstimator()
    except rospy.ROSInterruptException:
        pass
