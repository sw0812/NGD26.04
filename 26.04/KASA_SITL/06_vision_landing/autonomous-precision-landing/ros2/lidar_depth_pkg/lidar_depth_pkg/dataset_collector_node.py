#!/usr/bin/env python3
"""
Gazebo의 ground-truth 모델 위치(/gazebo/model_states)를 이용해서,
사람이 라벨링하지 않아도 depth map 위의 착륙패드 bounding box를
자동으로 계산하고, (depth map, YOLO 라벨) 쌍을 디스크에 저장하는 노드.

전제:
  - Gazebo 월드에 libgazebo_ros_state.so 플러그인이 켜져 있어서
    /gazebo/model_states (gazebo_msgs/ModelStates) 가 발행되고 있어야 함
  - 착륙패드 모델(landing_pad, 1m x 1m 사각형)이 월드에 스폰되어 있어야 함
  - depth_map_node가 /depth/image 를 발행 중이어야 함

저장물 (output_dir 아래):
  images/<seq>.npy   : float32 depth map 원본 (H,W)
  images/<seq>.png   : 8bit 정규화 미리보기용 이미지
  labels/<seq>.txt    : YOLO 포맷 라벨 "0 x_center y_center width height" (0~1 정규화)

한계 (다음 단계 개선 대상):
  - azimuth 랩어라운드(-pi/+pi 경계) 미처리
  - 착륙패드 자체의 회전(yaw)은 무시하고 world-axis-aligned 사각형으로 근사
  - 패드가 시야 밖이거나 다른 물체에 가려져도 ground-truth 기하학만으로 라벨을
    계산하기 때문에, 가려짐(occlusion) 여부는 반영하지 않음 (오탐 라벨 생성 가능)
"""

import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates


def quat_to_rotmat(x, y, z, w):
    """쿼터니언 -> 3x3 회전행렬 (world <- body 변환용)."""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ])


# lidar_link는 base_link 기준 pose: xyz=(0,0,-0.05), rpy=(0, pi/2, 0)
# (merge_iris_lidar.py에서 설정한 값과 반드시 일치해야 함)
_LIDAR_T = np.array([0.0, 0.0, -0.05])
_LIDAR_R = quat_to_rotmat(0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))  # Ry(90deg)


def world_point_to_lidar_frame(world_pt, drone_pos, drone_rotmat):
    """world 좌표계의 점을 lidar_link 좌표계로 변환."""
    body_vec = drone_rotmat.T @ (world_pt - drone_pos)
    lidar_vec = _LIDAR_R.T @ (body_vec - _LIDAR_T)
    return lidar_vec


def lidar_vec_to_pixel(vec, W, H, v_min, v_max):
    x, y, z = vec
    r = math.sqrt(x * x + y * y + z * z)
    if r < 1e-6:
        return None
    azimuth = math.atan2(y, x)
    elevation = math.asin(max(-1.0, min(1.0, z / r)))
    col = (azimuth + math.pi) / (2 * math.pi) * W
    row = (elevation - v_min) / (v_max - v_min) * H
    return col, row, r


class DatasetCollectorNode(Node):
    def __init__(self):
        super().__init__('dataset_collector_node')

        self.declare_parameter('drone_model_name', 'iris_lidar')
        self.declare_parameter('pad_model_name', 'landing_pad')
        self.declare_parameter('pad_size', 1.0)  # m, landing_pad.sdf의 box 크기와 일치
        self.declare_parameter('capture_interval_sec', 1.0)
        self.declare_parameter('output_dir', os.path.expanduser('~/lidar_dataset'))
        self.declare_parameter('horizontal_samples', 360)
        self.declare_parameter('vertical_samples', 64)
        self.declare_parameter('vertical_min_deg', -50.0)
        self.declare_parameter('vertical_max_deg', 50.0)
        # PNG 렌더링용 고정 depth 범위(m). 이 범위 밖은 검정으로 버린다.
        # 착륙 대상이 있는 근거리만 담아야 대비가 살아난다.
        self.declare_parameter('render_min', 0.5)
        self.declare_parameter('render_max', 12.0)

        self.drone_name = self.get_parameter('drone_model_name').value
        self.pad_name = self.get_parameter('pad_model_name').value
        self.pad_size = float(self.get_parameter('pad_size').value)
        self.capture_interval = float(self.get_parameter('capture_interval_sec').value)
        self.output_dir = self.get_parameter('output_dir').value
        self.W = int(self.get_parameter('horizontal_samples').value)
        self.H = int(self.get_parameter('vertical_samples').value)
        self.v_min = math.radians(self.get_parameter('vertical_min_deg').value)
        self.v_max = math.radians(self.get_parameter('vertical_max_deg').value)
        self.render_min = float(self.get_parameter('render_min').value)
        self.render_max = float(self.get_parameter('render_max').value)

        os.makedirs(os.path.join(self.output_dir, 'images'), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, 'labels'), exist_ok=True)

        qos_img = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )

        self.latest_depth = None  # (np.ndarray, header)
        self.latest_states = None  # ModelStates

        self.create_subscription(Image, '/depth/image', self.depth_cb, qos_img)
        self.create_subscription(
            ModelStates, '/gazebo/model_states', self.states_cb, qos_state
        )

        self.seq = 0
        self.saved_count = 0
        self.timer = self.create_timer(self.capture_interval, self.capture)

        self.get_logger().info(
            f'DatasetCollectorNode 시작: output={self.output_dir}, '
            f'interval={self.capture_interval}s'
        )

    def depth_cb(self, msg: Image):
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width
        ).copy()
        self.latest_depth = (depth, msg.header)

    def states_cb(self, msg: ModelStates):
        self.latest_states = msg

    def capture(self):
        if self.latest_depth is None or self.latest_states is None:
            self.get_logger().warn(
                'depth 또는 model_states 미수신 상태, 캡처 스킵', throttle_duration_sec=3.0
            )
            return

        names = list(self.latest_states.name)
        if self.drone_name not in names or self.pad_name not in names:
            self.get_logger().warn(
                f'모델 이름을 못 찾음 (drone={self.drone_name in names}, '
                f'pad={self.pad_name in names}). ModelStates 이름 목록: {names}',
                throttle_duration_sec=3.0,
            )
            return

        drone_pose = self.latest_states.pose[names.index(self.drone_name)]
        pad_pose = self.latest_states.pose[names.index(self.pad_name)]

        drone_pos = np.array([
            drone_pose.position.x, drone_pose.position.y, drone_pose.position.z
        ])
        drone_rotmat = quat_to_rotmat(
            drone_pose.orientation.x, drone_pose.orientation.y,
            drone_pose.orientation.z, drone_pose.orientation.w,
        )
        pad_center = np.array([
            pad_pose.position.x, pad_pose.position.y, pad_pose.position.z + self.pad_size * 0.075
        ])  # 패드 윗면 높이 보정(landing_pad.sdf의 link pose 0.075 참고)

        half = self.pad_size / 2.0
        corners_world = [
            pad_center + np.array([dx, dy, 0.0])
            for dx in (-half, half) for dy in (-half, half)
        ]

        pixels = []
        for c in corners_world:
            lidar_vec = world_point_to_lidar_frame(c, drone_pos, drone_rotmat)
            px = lidar_vec_to_pixel(lidar_vec, self.W, self.H, self.v_min, self.v_max)
            if px is not None:
                pixels.append(px)

        if len(pixels) < 4:
            self.get_logger().warn(
                f'코너 투영 실패 ({len(pixels)}/4개만 성공). '
                f'드론-패드 거리={np.linalg.norm(pad_center - drone_pos):.2f}m',
                throttle_duration_sec=2.0,
            )
            return  # 코너 중 하나라도 계산 실패하면 이번 프레임은 건너뜀

        cols = [p[0] for p in pixels]
        rows = [p[1] for p in pixels]
        col_min, col_max = max(0, min(cols)), min(self.W, max(cols))
        row_min, row_max = max(0, min(rows)), min(self.H, max(rows))

        # 화면 밖으로 완전히 벗어나거나 너무 작으면 스킵 (azimuth wrap 근처 등)
        if col_max - col_min < 1 or row_max - row_min < 1:
            self.get_logger().warn(
                f'패드가 시야 밖이거나 투영 실패, 스킵. '
                f'raw pixel cols={[f"{c:.1f}" for c in cols]} rows={[f"{r:.1f}" for r in rows]} '
                f'(H={self.H}, v_min={math.degrees(self.v_min):.1f}deg, v_max={math.degrees(self.v_max):.1f}deg), '
                f'드론-패드 거리={np.linalg.norm(pad_center - drone_pos):.2f}m',
                throttle_duration_sec=2.0,
            )
            return
        if (col_max - col_min) > self.W * 0.9:
            # azimuth 랩어라운드로 좌우가 뒤집혀 폭이 전체 이미지만큼 계산된 경우 -> 스킵
            return

        x_center = (col_min + col_max) / 2.0 / self.W
        y_center = (row_min + row_max) / 2.0 / self.H
        box_w = (col_max - col_min) / self.W
        box_h = (row_max - row_min) / self.H

        depth, header = self.latest_depth
        seq_str = f'{self.seq:06d}'
        np.save(os.path.join(self.output_dir, 'images', f'{seq_str}.npy'), depth)

        # 8bit 미리보기 PNG (선택 사항, 사람이 눈으로 확인하기 편하도록)
        # depth -> 8bit PNG 변환.
        # 프레임마다 min/max로 정규화하면 (a) 가장 가까운 물체(=패드)가 항상
        # 밝기 0으로 찍혀 빈 픽셀(hole)과 구분이 안 되고 (b) 멀리 있는 지면
        # (수십 m)이 스케일을 다 잡아먹어 근거리 대비가 사라진다.
        # -> 고정 범위(render_min~render_max)로 자르고, 가까울수록 밝게 반전.
        gray = self._depth_to_gray(depth, self.render_min, self.render_max)
        self._save_png(os.path.join(self.output_dir, 'images', f'{seq_str}.png'), gray)

        with open(os.path.join(self.output_dir, 'labels', f'{seq_str}.txt'), 'w') as f:
            f.write(f'0 {x_center:.6f} {y_center:.6f} {box_w:.6f} {box_h:.6f}\n')

        self.seq += 1
        self.saved_count += 1
        self.get_logger().info(
            f'[{self.saved_count}] 저장: {seq_str} '
            f'(bbox center=({x_center:.2f},{y_center:.2f}), size=({box_w:.2f},{box_h:.2f}))'
        )

    @staticmethod
    def _depth_to_gray(depth, render_min, render_max):
        """depth(float32, 0=hole) -> 8bit grayscale.
        render_min~render_max 범위로 자르고 가까울수록 밝게(255) 반전한다.
        hole(0)과 range 밖은 0(검정)으로 둬서 배경과 구분되게 한다."""
        valid = (depth >= render_min) & (depth <= render_max)
        norm = np.zeros_like(depth, dtype=np.float32)
        span = max(render_max - render_min, 1e-6)
        # 반전: render_min -> 1.0(밝음), render_max -> 0.0(어두움)
        norm[valid] = 1.0 - (depth[valid] - render_min) / span
        return (np.clip(norm, 0.0, 1.0) * 255).astype(np.uint8)

    @staticmethod
    def _save_png(path, gray_2d_uint8):
        """외부 의존성(cv2/PIL) 없이 순수 파이썬으로 8bit grayscale PNG 저장."""
        import struct
        import zlib

        h, w = gray_2d_uint8.shape

        def chunk(tag, data):
            return (
                struct.pack('>I', len(data)) + tag + data
                + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)
            )

        raw = b''.join(
            b'\x00' + gray_2d_uint8[y].tobytes() for y in range(h)
        )
        compressed = zlib.compress(raw, 9)

        png = b'\x89PNG\r\n\x1a\n'
        png += chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0))
        png += chunk(b'IDAT', compressed)
        png += chunk(b'IEND', b'')

        with open(path, 'wb') as f:
            f.write(png)


def main(args=None):
    rclpy.init(args=args)
    node = DatasetCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
