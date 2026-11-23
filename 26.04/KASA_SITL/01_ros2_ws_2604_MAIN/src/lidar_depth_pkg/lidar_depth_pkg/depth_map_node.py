#!/usr/bin/env python3
"""
LiDAR PointCloud2 -> 2D Depth Map (sensor_msgs/Image) 변환 노드.

/lidar/points (sensor_msgs/PointCloud2) 를 구독해서,
구면 투영(spherical projection)으로 2D 깊이 이미지를 만들고
/depth/image (sensor_msgs/Image, encoding=32FC1) 로 발행한다.

투영 격자 크기는 Gazebo에서 설정한 LiDAR 스펙(horizontal 360 samples,
vertical 16 samples, elevation +-15deg)과 맞춰뒀다. 실제 AlphaAir 10
스펙에 맞게 파라미터로 조정 가능.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2 as pc2


class DepthMapNode(Node):
    def __init__(self):
        super().__init__('depth_map_node')

        # ---- 파라미터 (Gazebo LiDAR 플러그인 설정과 맞춤) ----
        self.declare_parameter('input_topic', '/lidar/points')
        self.declare_parameter('output_topic', '/depth/image')
        self.declare_parameter('horizontal_samples', 360)
        self.declare_parameter('vertical_samples', 64)
        self.declare_parameter('vertical_min_deg', -50.0)
        self.declare_parameter('vertical_max_deg', 50.0)
        self.declare_parameter('range_min', 0.3)   # 이보다 가까운 포인트 제거
        self.declare_parameter('range_max', 50.0)  # 이보다 먼 포인트 제거

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.h_samples = int(self.get_parameter('horizontal_samples').value)
        self.v_samples = int(self.get_parameter('vertical_samples').value)
        self.v_min = math.radians(self.get_parameter('vertical_min_deg').value)
        self.v_max = math.radians(self.get_parameter('vertical_max_deg').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        # LiDAR 센서와 동일하게 BEST_EFFORT/VOLATILE QoS 사용
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, qos
        )
        self.pub = self.create_publisher(Image, self.output_topic, 10)

        self.get_logger().info(
            f'DepthMapNode 시작: {self.input_topic} -> {self.output_topic} '
            f'({self.h_samples}x{self.v_samples})'
        )

    def cloud_callback(self, msg: PointCloud2):
        # PointCloud2 -> (N,3) numpy array (x,y,z)
        points = np.array(
            list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)),
            dtype=np.float32,
        )

        # 깊이맵 초기화: 0.0 = "포인트 없음(hole)"
        depth_img = np.zeros((self.v_samples, self.h_samples), dtype=np.float32)

        if points.shape[0] == 0:
            self._publish(depth_img, msg)
            return

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        r = np.sqrt(x * x + y * y + z * z)

        # 1) 범위 필터링 (관측 범위 밖 포인트 제거)
        valid = (r >= self.range_min) & (r <= self.range_max) & (r > 1e-6)
        x, y, z, r = x[valid], y[valid], z[valid], r[valid]

        if r.shape[0] == 0:
            self._publish(depth_img, msg)
            return

        # 2) 구면 좌표로 투영
        azimuth = np.arctan2(y, x)  # [-pi, pi]
        elevation = np.arcsin(np.clip(z / r, -1.0, 1.0))  # [v_min, v_max] 근처

        # 3) 픽셀 인덱스로 매핑
        col = ((azimuth + math.pi) / (2 * math.pi) * self.h_samples).astype(np.int32)
        col = np.clip(col, 0, self.h_samples - 1)

        row = ((elevation - self.v_min) / (self.v_max - self.v_min) * self.v_samples).astype(np.int32)
        in_fov = (row >= 0) & (row < self.v_samples)
        col, row, r = col[in_fov], row[in_fov], r[in_fov]

        # 4) z-buffer 방식으로 최근접(가장 가까운) 거리만 저장
        #    같은 픽셀에 여러 포인트가 매핑될 때, 더 가까운 값으로 덮어씀
        flat_idx = row * self.h_samples + col
        order = np.argsort(-r)  # 먼 것부터 써서, 가까운 게 마지막에 덮어쓰도록
        flat_depth = depth_img.reshape(-1)
        flat_depth[flat_idx[order]] = r[order]
        depth_img = flat_depth.reshape(self.v_samples, self.h_samples)

        # TODO(추후 개선): 빈 픽셀(hole) 보간 — 최근접 이웃 보간 또는 median filter.
        # 계획서 3장의 "깊이맵 보정" 단계에 해당. 지금은 스켈레톤이라 생략.

        self._publish(depth_img, msg)

    def _publish(self, depth_img: np.ndarray, src_msg: PointCloud2):
        img_msg = Image()
        img_msg.header = src_msg.header
        img_msg.height, img_msg.width = depth_img.shape
        img_msg.encoding = '32FC1'
        img_msg.is_bigendian = 0
        img_msg.step = img_msg.width * 4  # float32 = 4 bytes
        img_msg.data = depth_img.astype(np.float32).tobytes()
        self.pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
