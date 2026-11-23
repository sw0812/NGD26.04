#!/usr/bin/env python3
"""
/depth/image(32FC1) 구독 -> 학습 때와 동일한 clip+invert로 그레이스케일 렌더링
-> YOLO(best.pt)로 착륙패드 검출 -> bbox 중심 픽셀의 실제 depth 값으로 3D 역투영
-> lidar_link -> base_link -> FRD -> local NED 좌표로 변환해서
/landing_target (geometry_msgs/PointStamped, frame_id='map', local NED 절대좌표)로 발행.

검출 안 되거나 신뢰도 낮으면 발행하지 않음 -> landing_control_node가
마지막 수신 시각 기준으로 "마커 유실" 판단.
"""

import math

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

from ultralytics import YOLO
from px4_msgs.msg import VehicleLocalPosition

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class YoloInferenceNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_node')

        self.declare_parameter('weights_path', '/home/sasllab/Downloads/best.pt')
        self.declare_parameter('depth_topic', '/depth/image')
        self.declare_parameter('local_pos_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('output_topic', '/landing_target')
        self.declare_parameter('conf_threshold', 0.4)

        # depth_map_node와 동일한 격자/투영 파라미터 (반드시 맞춰야 함)
        self.declare_parameter('h_samples', 360)
        self.declare_parameter('v_samples', 64)
        self.declare_parameter('v_min_deg', -50.0)
        self.declare_parameter('v_max_deg', 50.0)

        # dataset_collector_node와 동일한 렌더링 파라미터
        self.declare_parameter('render_min', 0.5)
        self.declare_parameter('render_max', 12.0)

        # LiDAR 장착 오프셋 (iris_lidar.sdf 기준, base_link 대비)
        self.declare_parameter('lidar_offset_x', 0.0)
        self.declare_parameter('lidar_offset_y', 0.0)
        self.declare_parameter('lidar_offset_z', -0.05)

        weights_path = self.get_parameter('weights_path').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.local_pos_topic = self.get_parameter('local_pos_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)

        self.h_samples = int(self.get_parameter('h_samples').value)
        self.v_samples = int(self.get_parameter('v_samples').value)
        self.v_min = math.radians(self.get_parameter('v_min_deg').value)
        self.v_max = math.radians(self.get_parameter('v_max_deg').value)

        self.render_min = float(self.get_parameter('render_min').value)
        self.render_max = float(self.get_parameter('render_max').value)

        self.lidar_off_x = float(self.get_parameter('lidar_offset_x').value)
        self.lidar_off_y = float(self.get_parameter('lidar_offset_y').value)
        self.lidar_off_z = float(self.get_parameter('lidar_offset_z').value)

        self.get_logger().info(f'YOLO 가중치 로드 중: {weights_path}')
        self.model = YOLO(weights_path)
        self.get_logger().info('YOLO 로드 완료')

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.drone_yaw = 0.0
        self.have_local_pos = False

        qos_img = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        self.create_subscription(Image, self.depth_topic, self.depth_cb, qos_img)
        self.create_subscription(
            VehicleLocalPosition, self.local_pos_topic, self.local_pos_cb, PX4_QOS
        )
        self.target_pub = self.create_publisher(PointStamped, self.output_topic, 10)

        self.get_logger().info(
            f'YoloInferenceNode 시작: {self.depth_topic} -> {self.output_topic}'
        )

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.drone_x = msg.x
        self.drone_y = msg.y
        self.drone_z = msg.z
        self.drone_yaw = getattr(msg, 'heading', 0.0)
        self.have_local_pos = True

    def render_depth_image(self, depth: np.ndarray) -> np.ndarray:
        """dataset_collector_node와 동일한 clip+invert 렌더링.
        depth==0(hole)은 배경(검정, 0)으로 처리, 가까울수록 밝게."""
        clipped = np.clip(depth, self.render_min, self.render_max)
        normalized = (self.render_max - clipped) / (self.render_max - self.render_min)
        img = (normalized * 255.0).astype(np.uint8)
        img[depth <= 1e-6] = 0
        return img

    def pixel_to_depth_value(self, depth: np.ndarray, col: float, row: float):
        """bbox 중심 픽셀 주변 3x3에서 유효한(0이 아닌) depth의 중앙값을 취함."""
        r0, c0 = int(round(row)), int(round(col))
        r_lo, r_hi = max(0, r0 - 1), min(depth.shape[0], r0 + 2)
        c_lo, c_hi = max(0, c0 - 1), min(depth.shape[1], c0 + 2)
        patch = depth[r_lo:r_hi, c_lo:c_hi]
        valid = patch[patch > 1e-6]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def pixel_to_lidar_xyz(self, col: float, row: float, depth_value: float):
        azimuth = (col + 0.5) / self.h_samples * (2 * math.pi) - math.pi
        elevation = self.v_min + (row + 0.5) / self.v_samples * (self.v_max - self.v_min)
        horizontal_r = depth_value * math.cos(elevation)
        x = horizontal_r * math.cos(azimuth)
        y = horizontal_r * math.sin(azimuth)
        z = depth_value * math.sin(elevation)
        return x, y, z

    def lidar_to_ned(self, x_lidar, y_lidar, z_lidar):
        # lidar_link -> base_link (평행이동만, 회전 없음)
        x_base = x_lidar + self.lidar_off_x
        y_base = y_lidar + self.lidar_off_y
        z_base = z_lidar + self.lidar_off_z

        # base_link(Gazebo ENU-body) -> FRD
        x_frd = x_base
        y_frd = -y_base
        z_frd = -z_base

        # FRD -> local NED (드론 현재 위치/heading 기준 회전 + 평행이동)
        yaw = self.drone_yaw
        dx = x_frd * math.cos(yaw) - y_frd * math.sin(yaw)
        dy = x_frd * math.sin(yaw) + y_frd * math.cos(yaw)
        dz = z_frd

        target_x = self.drone_x + dx
        target_y = self.drone_y + dy
        target_z = self.drone_z + dz
        return target_x, target_y, target_z

    def depth_cb(self, msg: Image):
        if not self.have_local_pos:
            return

        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width
        ).copy()

        render = self.render_depth_image(depth)
        render_bgr = cv2.cvtColor(render, cv2.COLOR_GRAY2BGR)

        results = self.model.predict(
            render_bgr, conf=self.conf_threshold, verbose=False
        )

        if len(results) == 0 or len(results[0].boxes) == 0:
            self.get_logger().info('YOLO 검출 없음')
            self.get_logger().info('YOLO 검출 없음')
            return

        boxes = results[0].boxes
        best_idx = int(boxes.conf.argmax())
        best_box = boxes.xywh[best_idx].cpu().numpy()  # cx, cy, w, h (픽셀 단위)
        best_conf = float(boxes.conf[best_idx])
        cx, cy = float(best_box[0]), float(best_box[1])

        depth_value = self.pixel_to_depth_value(depth, cx, cy)
        if depth_value is None:
            self.get_logger().warn('검출은 됐으나 해당 픽셀에 유효 depth 없음, 스킵')
            return

        x_lidar, y_lidar, z_lidar = self.pixel_to_lidar_xyz(cx, cy, depth_value)
        target_x, target_y, target_z = self.lidar_to_ned(x_lidar, y_lidar, z_lidar)

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.point.x = target_x
        out.point.y = target_y
        out.point.z = target_z
        self.target_pub.publish(out)

        self.get_logger().info(
            f'검출 conf={best_conf:.2f} pixel=({cx:.0f},{cy:.0f}) depth={depth_value:.2f}m '
            f'-> NED=({target_x:.2f},{target_y:.2f},{target_z:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = YoloInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
