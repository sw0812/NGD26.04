#!/usr/bin/env python3
"""
/depth/image(32FC1) 구독 -> LiDAR 격자에서 "장애물"로 분류된 픽셀들의
포텐셜필드 반발 벡터를 계산해서 /avoidance_velocity(Vector3Stamped, FRD)로 발행.

지면/착륙패드는 다음 두 조건 중 하나라도 만족하면 회피 대상에서 제외:
  1) 센서보다 ground_margin(m) 이상 아래에 있음 (일반적인 지면)
  2) 수평거리가 safe_horizontal_radius(m) 이내 (착륙지점 바로 아래 -
     착지 막바지에 높이차가 줄어들어도 패드를 장애물로 오인하지 않도록)
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped


class ObstacleRepulsionNode(Node):
    def __init__(self):
        super().__init__('obstacle_repulsion_node')

        self.declare_parameter('depth_topic', '/depth/image')
        self.declare_parameter('output_topic', '/avoidance_velocity')

        # depth_map_node와 동일한 격자 파라미터 (반드시 맞춰야 함)
        self.declare_parameter('h_samples', 360)
        self.declare_parameter('v_samples', 64)
        self.declare_parameter('v_min_deg', -50.0)
        self.declare_parameter('v_max_deg', 50.0)

        # 회피 파라미터
        self.declare_parameter('avoid_radius', 2.0)
        self.declare_parameter('ground_margin', 0.3)
        self.declare_parameter('safe_horizontal_radius', 1.0)
        self.declare_parameter('avoid_gain', 1.2)
        self.declare_parameter('max_avoid_speed', 1.5)
        self.declare_parameter('range_min', 0.3)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.h_samples = int(self.get_parameter('h_samples').value)
        self.v_samples = int(self.get_parameter('v_samples').value)
        self.v_min = math.radians(self.get_parameter('v_min_deg').value)
        self.v_max = math.radians(self.get_parameter('v_max_deg').value)

        self.avoid_radius = float(self.get_parameter('avoid_radius').value)
        self.ground_margin = float(self.get_parameter('ground_margin').value)
        self.safe_horizontal_radius = float(
            self.get_parameter('safe_horizontal_radius').value
        )
        self.avoid_gain = float(self.get_parameter('avoid_gain').value)
        self.max_avoid_speed = float(self.get_parameter('max_avoid_speed').value)
        self.range_min = float(self.get_parameter('range_min').value)

        # ---- 격자 방향코사인 사전계산 (한 번만) ----
        rows = np.arange(self.v_samples)
        cols = np.arange(self.h_samples)
        elevation = self.v_min + (rows + 0.5) / self.v_samples * (self.v_max - self.v_min)
        azimuth = (cols + 0.5) / self.h_samples * (2 * math.pi) - math.pi

        elev_grid, az_grid = np.meshgrid(elevation, azimuth, indexing='ij')
        self.dir_x = np.cos(elev_grid) * np.cos(az_grid)
        self.dir_y = np.cos(elev_grid) * np.sin(az_grid)
        self.dir_z = np.sin(elev_grid)

        qos_img = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        self.create_subscription(Image, self.depth_topic, self.depth_cb, qos_img)
        self.pub = self.create_publisher(Vector3Stamped, self.output_topic, 10)

        self.get_logger().info(
            f'ObstacleRepulsionNode 시작: avoid_radius={self.avoid_radius}m, '
            f'ground_margin={self.ground_margin}m, '
            f'safe_horizontal_radius={self.safe_horizontal_radius}m'
        )

    def depth_cb(self, msg: Image):
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width
        )

        pz_grid = self.dir_z * depth
        px_grid = self.dir_x * depth
        py_grid = self.dir_y * depth
        horiz_dist = np.sqrt(px_grid ** 2 + py_grid ** 2)

        is_ground_like = (horiz_dist <= self.safe_horizontal_radius)
        valid = (
            (depth >= self.range_min)
            & (depth <= self.avoid_radius)
            & (~is_ground_like)
        )

        out = Vector3Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link_frd'

        if not np.any(valid):
            out.vector.x, out.vector.y, out.vector.z = 0.0, 0.0, 0.0
            self.pub.publish(out)
            return

        d = depth[valid]
        px = px_grid[valid]
        py = py_grid[valid]
        pz = pz_grid[valid]

        dist = np.maximum(d, 1e-3)
        strength = self.avoid_gain * (1.0 / dist - 1.0 / self.avoid_radius)
        strength = np.maximum(strength, 0.0)

        rep_x = np.sum(-(px / dist) * strength)
        rep_y = np.sum(-(py / dist) * strength)
        rep_z = np.sum(-(pz / dist) * strength)

        # lidar_link(Gazebo ENU-body) -> FRD
        rep_x_frd = rep_x
        rep_y_frd = -rep_y
        rep_z_frd = 0.0  # 수직 회피 사용 안 함(상승 폭주 방지)

        speed = math.sqrt(rep_x_frd ** 2 + rep_y_frd ** 2 + rep_z_frd ** 2)
        if speed > self.max_avoid_speed:
            scale = self.max_avoid_speed / speed
            rep_x_frd *= scale
            rep_y_frd *= scale
            rep_z_frd *= scale

        out.vector.x = float(rep_x_frd)
        out.vector.y = float(rep_y_frd)
        out.vector.z = float(rep_z_frd)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleRepulsionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
