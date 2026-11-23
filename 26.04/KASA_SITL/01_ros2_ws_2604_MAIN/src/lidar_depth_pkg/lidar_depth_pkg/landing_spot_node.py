#!/usr/bin/env python3
"""
/depth/image (32FC1) 를 분석해서 "가장 평평하고 가까운" 착륙 후보 지점을
찾아 /landing/candidate_point (geometry_msgs/PointStamped, lidar_link 기준
상대 좌표) 로 발행하는 룰 기반(rule-based) 노드.

방법: depth map을 가로(azimuth) 방향으로 슬라이딩 윈도우로 훑으면서,
각 윈도우 안에서
  - 유효 픽셀 비율(valid_frac)이 충분히 높고
  - 표준편차(std)가 가장 낮고 (=거리 변화가 적다=평평하다)
  - 평균 거리가 range_max 이내인
윈도우를 "착륙 후보"로 선택한다.

적분 이미지(summed-area table)를 써서 윈도우 합/제곱합을 O(1)에 구하는
방식으로, 매 프레임 슬라이딩 윈도우 계산을 벡터화했다.

한계 (다음 단계 개선 대상):
  - 방위각(azimuth) 랩어라운드(-pi/+pi 경계) 처리는 생략함
  - elevation은 윈도우 중심을 0(수평)으로 근사함 (실제 최적 elevation
    분포를 반영하지 않음)
  - 진짜 "평지"인지 판단하려면 표준편차뿐 아니라 지면 법선(normal) 추정이
    필요하지만, 여기서는 스켈레톤 수준으로 표준편차만 사용함
"""

import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped


class LandingSpotNode(Node):
    def __init__(self):
        super().__init__('landing_spot_node')

        self.declare_parameter('input_topic', '/depth/image')
        self.declare_parameter('output_topic', '/landing/candidate_point')
        self.declare_parameter('horizontal_window', 30)  # px (~30도)
        self.declare_parameter('min_valid_fraction', 0.6)
        self.declare_parameter('range_max_for_landing', 10.0)  # m
        self.declare_parameter('step', 5)  # 슬라이딩 스텝(px)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.hw = int(self.get_parameter('horizontal_window').value)
        self.min_valid_fraction = float(self.get_parameter('min_valid_fraction').value)
        self.range_max = float(self.get_parameter('range_max_for_landing').value)
        self.step = int(self.get_parameter('step').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(
            Image, self.input_topic, self.image_callback, qos
        )
        self.pub = self.create_publisher(PointStamped, self.output_topic, 10)

        self.get_logger().info(
            f'LandingSpotNode 시작: {self.input_topic} -> {self.output_topic} '
            f'(window={self.hw}px, min_valid={self.min_valid_fraction})'
        )
        self._last_status_len = 0

    def status_line(self, text: str):
        """한 줄(캐리지리턴)로 계속 덮어써서 갱신하는 상태 출력.
        ROS2 logger는 매번 개행이 붙어서 도배되니 stdout에 직접 씀."""
        pad = max(self._last_status_len - len(text), 0)
        sys.stdout.write('\r' + text + (' ' * pad))
        sys.stdout.flush()
        self._last_status_len = len(text)

    def image_callback(self, msg: Image):
        if msg.encoding != '32FC1':
            self.get_logger().warn(f'예상치 못한 encoding: {msg.encoding}')
            return

        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width
        ).astype(np.float64)
        H, W = depth.shape

        if W < self.hw:
            self.get_logger().warn('이미지 폭이 윈도우보다 작습니다. 파라미터를 확인하세요.')
            return

        valid = (depth > 0.0).astype(np.float64)
        vals = depth * valid
        sq = (depth * depth) * valid

        # ---- 적분 이미지(summed-area table), 세로는 항상 전체(H) 사용 ----
        col_sum_vals = np.sum(vals, axis=0)      # (W,)
        col_sum_sq = np.sum(sq, axis=0)          # (W,)
        col_sum_valid = np.sum(valid, axis=0)    # (W,)

        sat_vals = np.concatenate(([0.0], np.cumsum(col_sum_vals)))
        sat_sq = np.concatenate(([0.0], np.cumsum(col_sum_sq)))
        sat_valid = np.concatenate(([0.0], np.cumsum(col_sum_valid)))

        best_score = None
        best_x0 = None
        best_mean = None

        for x0 in range(0, W - self.hw + 1, self.step):
            x1 = x0 + self.hw
            sum_vals = sat_vals[x1] - sat_vals[x0]
            sum_sq = sat_sq[x1] - sat_sq[x0]
            count = sat_valid[x1] - sat_valid[x0]

            total_px = H * self.hw
            valid_frac = count / total_px if total_px > 0 else 0.0
            if valid_frac < self.min_valid_fraction or count < 1:
                continue

            mean = sum_vals / count
            var = max(sum_sq / count - mean * mean, 0.0)
            std = math.sqrt(var)

            if mean > self.range_max:
                continue

            # 점수: 표준편차(평탄도)가 우선, 동률이면 더 가까운 쪽 선호
            score = std
            if best_score is None or score < best_score:
                best_score = score
                best_x0 = x0
                best_mean = mean

        if best_x0 is None:
            # 조건을 만족하는 평지 후보를 못 찾음 (한 줄 상태로만 표시, 발행 안 함)
            self.status_line('[LANDING] 후보 없음 (평지 조건 불만족 또는 range 초과)')
            return

        center_x = best_x0 + self.hw / 2.0
        azimuth = (center_x / W) * 2 * math.pi - math.pi
        elevation = 0.0  # 근사치 (TODO: 윈도우 내 실제 elevation 분포 반영)

        dx = math.cos(elevation) * math.cos(azimuth)
        dy = math.cos(elevation) * math.sin(azimuth)
        dz = math.sin(elevation)

        point_msg = PointStamped()
        point_msg.header = msg.header
        point_msg.point.x = dx * best_mean
        point_msg.point.y = dy * best_mean
        point_msg.point.z = dz * best_mean
        self.pub.publish(point_msg)

        self.status_line(
            f'[LANDING] azimuth={math.degrees(azimuth):+6.1f}deg  '
            f'dist={best_mean:5.2f}m  std={best_score:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LandingSpotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
