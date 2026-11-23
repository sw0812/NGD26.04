#!/usr/bin/env python3
"""
dataset_collector_node.py의 v2. mAP=0 원인 리뷰에서 발견된 문제를 고친다.

v1 대비 변경점:
  1. /depth/image 와 /gazebo/model_states를 message_filters로 근사 시간동기화.
     (v1은 "가장 최근값"을 그냥 짝지어서, 드론이 이동 중인 프레임은
      depth 캡처 시점과 pose 시점이 어긋날 수 있었음)
  2. azimuth를 원형(circular) 통계로 unwrap해서 -pi/+pi 경계를 넘는
     bbox도 올바르게 계산 (v1은 경계 넘는 경우 통째로 skip만 함)
  3. self-check: 계산된 bbox 중심 픽셀의 "실측 depth"와 "패드까지 계산 거리"를
     비교해서 크게 다르면 프레임을 버림. 외부파라미터(_LIDAR_T/_LIDAR_R)가
     잘못됐거나 occlusion이 있으면 이 체크에서 걸러진다.
  4. LiDAR 외부파라미터를 파라미터화(lidar_xyz, lidar_rpy)해서 코드 수정 없이
     캘리브레이션 가능하게 함. lidar_calib_check.py로 먼저 검증할 것.
  5. pad 상단 오프셋을 pad_size에 곱하는 매직넘버 대신 고정 파라미터로 분리.

주의: 이 파일은 v1의 좌표 변환 공식 자체(atan2/asin 기반 구면 투영)는
그대로 유지한다 - 그건 depth_map_node.py와 동일해서 문제가 아니었음.
문제였던 건 "world->lidar 변환에 쓰는 외부파라미터가 맞는가"였고,
그건 이 노드만으로 확정할 수 없으므로 self-check로 런타임에 걸러낸다.
"""

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
import message_filters


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


def rotmat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def world_point_to_lidar_frame(world_pt, drone_pos, drone_rotmat, lidar_t, lidar_r):
    """world 좌표계의 점을 lidar 프레임으로 변환."""
    body_vec = drone_rotmat.T @ (world_pt - drone_pos)
    lidar_vec = lidar_r.T @ (body_vec - lidar_t)
    return lidar_vec


def lidar_vec_to_azel(vec):
    x, y, z = vec
    r = math.sqrt(x * x + y * y + z * z)
    if r < 1e-6:
        return None
    azimuth = math.atan2(y, x)
    elevation = math.asin(max(-1.0, min(1.0, z / r)))
    return azimuth, elevation, r


def azel_to_pixel(azimuth, elevation, W, H, v_min, v_max):
    col = (azimuth + math.pi) / (2 * math.pi) * W
    row = (elevation - v_min) / (v_max - v_min) * H
    return col, row


def circular_mean(angles):
    s = sum(math.sin(a) for a in angles)
    c = sum(math.cos(a) for a in angles)
    return math.atan2(s, c)


def unwrap_around(angles, ref):
    """ref 근처가 되도록 각 angle을 2pi 배수로 이동시켜 선형 min/max가 가능하게 함."""
    out = []
    for a in angles:
        d = a - ref
        d = (d + math.pi) % (2 * math.pi) - math.pi  # [-pi, pi)로 wrap
        out.append(ref + d)
    return out


class DatasetCollectorNodeV2(Node):
    def __init__(self):
        super().__init__('dataset_collector_node_v2')

        self.declare_parameter('drone_model_name', 'iris_lidar')
        self.declare_parameter('pad_model_name', 'landing_pad')
        self.declare_parameter('pad_size', 1.0)
        self.declare_parameter('pad_top_offset_m', 0.075)  # landing_pad.sdf link pose 기준 고정값
        self.declare_parameter('capture_interval_sec', 1.0)
        self.declare_parameter('output_dir', os.path.expanduser('~/lidar_dataset_v2'))
        self.declare_parameter('horizontal_samples', 360)
        self.declare_parameter('vertical_samples', 64)
        self.declare_parameter('vertical_min_deg', -50.0)
        self.declare_parameter('vertical_max_deg', 50.0)
        self.declare_parameter('render_min', 0.5)
        self.declare_parameter('render_max', 12.0)
        # base_link -> lidar 프레임 외부파라미터. lidar_calib_check.py로 먼저 검증할 것.
        self.declare_parameter('lidar_xyz', [0.0, 0.0, -0.05])
        self.declare_parameter('lidar_rpy_deg', [0.0, 90.0, 0.0])
        # self-check: 계산 거리 vs 실측 depth 허용 오차
        self.declare_parameter('range_check_tol_m', 0.5)
        # depth/model_states 시간 동기화 허용 오차
        self.declare_parameter('sync_slop_sec', 0.05)

        self.drone_name = self.get_parameter('drone_model_name').value
        self.pad_name = self.get_parameter('pad_model_name').value
        self.pad_size = float(self.get_parameter('pad_size').value)
        self.pad_top_offset = float(self.get_parameter('pad_top_offset_m').value)
        self.capture_interval = float(self.get_parameter('capture_interval_sec').value)
        self.output_dir = self.get_parameter('output_dir').value
        self.W = int(self.get_parameter('horizontal_samples').value)
        self.H = int(self.get_parameter('vertical_samples').value)
        self.v_min = math.radians(self.get_parameter('vertical_min_deg').value)
        self.v_max = math.radians(self.get_parameter('vertical_max_deg').value)
        self.render_min = float(self.get_parameter('render_min').value)
        self.render_max = float(self.get_parameter('render_max').value)
        self.range_check_tol = float(self.get_parameter('range_check_tol_m').value)

        lidar_xyz = self.get_parameter('lidar_xyz').value
        lidar_rpy_deg = self.get_parameter('lidar_rpy_deg').value
        self.lidar_t = np.array([float(v) for v in lidar_xyz])
        self.lidar_r = rotmat_from_rpy(*[math.radians(float(v)) for v in lidar_rpy_deg])

        os.makedirs(os.path.join(self.output_dir, 'images'), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, 'labels'), exist_ok=True)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10,
        )

        depth_sub = message_filters.Subscriber(self, Image, '/depth/image', qos_profile=qos)
        states_sub = message_filters.Subscriber(self, ModelStates, '/gazebo/model_states', qos_profile=qos)
        slop = float(self.get_parameter('sync_slop_sec').value)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [depth_sub, states_sub], queue_size=10, slop=slop
        )
        self.sync.registerCallback(self.synced_cb)

        self.last_capture_time = self.get_clock().now()
        self.seq = 0
        self.saved_count = 0
        self.rejected_count = 0

        self.get_logger().info(
            f'DatasetCollectorNodeV2 시작: output={self.output_dir}, '
            f'lidar_t={self.lidar_t.tolist()}, lidar_rpy_deg={lidar_rpy_deg}, '
            f'sync_slop={slop}s, range_check_tol={self.range_check_tol}m'
        )

    def synced_cb(self, depth_msg: Image, states_msg: ModelStates):
        now = self.get_clock().now()
        if (now - self.last_capture_time).nanoseconds < self.capture_interval * 1e9:
            return
        self.last_capture_time = now

        depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(
            depth_msg.height, depth_msg.width
        ).copy()

        names = list(states_msg.name)
        if self.drone_name not in names or self.pad_name not in names:
            self.get_logger().warn(
                f'모델 이름을 못 찾음 (drone={self.drone_name in names}, '
                f'pad={self.pad_name in names})', throttle_duration_sec=3.0,
            )
            return

        drone_pose = states_msg.pose[names.index(self.drone_name)]
        pad_pose = states_msg.pose[names.index(self.pad_name)]

        drone_pos = np.array([drone_pose.position.x, drone_pose.position.y, drone_pose.position.z])
        drone_rotmat = quat_to_rotmat(
            drone_pose.orientation.x, drone_pose.orientation.y,
            drone_pose.orientation.z, drone_pose.orientation.w,
        )
        pad_center = np.array([
            pad_pose.position.x, pad_pose.position.y,
            pad_pose.position.z + self.pad_top_offset,
        ])

        half = self.pad_size / 2.0
        corners_world = [
            pad_center + np.array([dx, dy, 0.0])
            for dx in (-half, half) for dy in (-half, half)
        ]

        azels = []
        for c in corners_world:
            vec = world_point_to_lidar_frame(c, drone_pos, drone_rotmat, self.lidar_t, self.lidar_r)
            azel = lidar_vec_to_azel(vec)
            if azel is not None:
                azels.append(azel)

        if len(azels) < 4:
            self._reject(f'코너 투영 실패 ({len(azels)}/4)')
            return

        # azimuth wraparound: 첫 코너를 기준으로 나머지를 원형 unwrap 후 선형 min/max
        center_center = world_point_to_lidar_frame(pad_center, drone_pos, drone_rotmat, self.lidar_t, self.lidar_r)
        center_azel = lidar_vec_to_azel(center_center)
        ref_az = center_azel[0] if center_azel is not None else azels[0][0]
        az_unwrapped = unwrap_around([a[0] for a in azels], ref_az)
        elevations = [a[1] for a in azels]
        ranges = [a[2] for a in azels]

        cols, rows = [], []
        for az, el in zip(az_unwrapped, elevations):
            cols.append(az)
            rows.append(azel_to_pixel(az, el, self.W, self.H, self.v_min, self.v_max)[1])

        az_min, az_max = min(cols), max(cols)
        row_min, row_max = max(0.0, min(rows)), min(float(self.H), max(rows))

        if (az_max - az_min) > math.radians(170):
            self._reject('unwrap 후에도 azimuth 폭이 비정상적으로 큼 (기하 오류 의심)')
            return

        col_min_f = (az_min + math.pi) / (2 * math.pi) * self.W
        col_max_f = (az_max + math.pi) / (2 * math.pi) * self.W
        col_min = col_min_f % self.W
        col_max = col_max_f % self.W
        if col_max < col_min:
            col_min, col_max = col_max, col_min

        if col_max - col_min < 1 or row_max - row_min < 1:
            self._reject('bbox가 시야 밖이거나 크기가 0에 가까움')
            return

        x_center = (col_min + col_max) / 2.0 / self.W
        y_center = (row_min + row_max) / 2.0 / self.H
        box_w = (col_max - col_min) / self.W
        box_h = (row_max - row_min) / self.H

        center_col_px = int(np.clip((col_min + col_max) / 2.0, 0, self.W - 1))
        center_row_px = int(np.clip((row_min + row_max) / 2.0, 0, self.H - 1))
        measured = depth[center_row_px, center_col_px]
        expected_r = float(np.mean(ranges))
        if measured <= 0.0:
            self._reject(
                f'bbox 중심 픽셀에 depth 관측값이 없음(hole) '
                f'(계산거리={expected_r:.2f}m) - 외부파라미터 불일치 또는 occlusion 의심'
            )
            return
        if abs(measured - expected_r) > self.range_check_tol:
            self._reject(
                f'거리 불일치: 계산={expected_r:.2f}m 실측={measured:.2f}m '
                f'(허용오차={self.range_check_tol}m) - occlusion 또는 라이다 외부파라미터 오류 의심'
            )
            return

        seq_str = f'{self.seq:06d}'
        np.save(os.path.join(self.output_dir, 'images', f'{seq_str}.npy'), depth)
        gray = self._depth_to_gray(depth, self.render_min, self.render_max)
        self._save_png(os.path.join(self.output_dir, 'images', f'{seq_str}.png'), gray)
        with open(os.path.join(self.output_dir, 'labels', f'{seq_str}.txt'), 'w') as f:
            f.write(f'0 {x_center:.6f} {y_center:.6f} {box_w:.6f} {box_h:.6f}\n')

        self.seq += 1
        self.saved_count += 1
        self.get_logger().info(
            f'[{self.saved_count} saved / {self.rejected_count} rejected] {seq_str} '
            f'bbox=({x_center:.2f},{y_center:.2f},{box_w:.2f},{box_h:.2f}) r={expected_r:.2f}m'
        )

    def _reject(self, reason):
        self.rejected_count += 1
        self.get_logger().warn(f'프레임 거부: {reason}', throttle_duration_sec=2.0)
        if self.rejected_count > 30 and self.saved_count == 0:
            self.get_logger().error(
                '30 프레임 연속 저장 실패 없음. lidar_xyz/lidar_rpy_deg 파라미터가 '
                '실제 SDF 구조와 맞지 않을 가능성이 매우 높습니다. '
                'lidar_calib_check 노드로 먼저 캘리브레이션하세요.',
                throttle_duration_sec=10.0,
            )

    @staticmethod
    def _depth_to_gray(depth, render_min, render_max):
        valid = (depth >= render_min) & (depth <= render_max)
        norm = np.zeros_like(depth, dtype=np.float32)
        span = max(render_max - render_min, 1e-6)
        norm[valid] = 1.0 - (depth[valid] - render_min) / span
        return (np.clip(norm, 0.0, 1.0) * 255).astype(np.uint8)

    @staticmethod
    def _save_png(path, gray_2d_uint8):
        import struct
        import zlib

        h, w = gray_2d_uint8.shape

        def chunk(tag, data):
            return (
                struct.pack('>I', len(data)) + tag + data
                + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)
            )

        raw = b''.join(b'\x00' + gray_2d_uint8[y].tobytes() for y in range(h))
        compressed = zlib.compress(raw, 9)

        png = b'\x89PNG\r\n\x1a\n'
        png += chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0))
        png += chunk(b'IDAT', compressed)
        png += chunk(b'IEND', b'')

        with open(path, 'wb') as f:
            f.write(png)


def main(args=None):
    rclpy.init(args=args)
    node = DatasetCollectorNodeV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
