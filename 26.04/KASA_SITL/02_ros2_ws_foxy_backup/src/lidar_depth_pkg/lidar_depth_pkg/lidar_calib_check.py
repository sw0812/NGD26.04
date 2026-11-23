#!/usr/bin/env python3
"""
LiDAR 외부파라미터(_LIDAR_T, _LIDAR_R) 검증용 진단 노드.

dataset_collector_node가 world 좌표 -> lidar 프레임으로 변환해 계산한
착륙패드 방향(azimuth/elevation)이, 실제 depth 이미지에서 관측되는
"가장 가까운 물체(패드로 추정)"의 방향과 일치하는지 비교한다.

두 값이 크게 어긋나면 _LIDAR_T/_LIDAR_R (base_link -> lidar 프레임 외부파라미터)
자체가 SDF 구조와 맞지 않는다는 뜻이므로, dataset_collector_node의 좌표계
가정을 다시 캘리브레이션해야 한다.

사용법:
  ros2 run lidar_depth_pkg lidar_calib_check
  (iris_lidar가 착륙패드를 바라보게 호버링 중일 때 실행)
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates

from lidar_depth_pkg.dataset_collector_node import (
    quat_to_rotmat,
    world_point_to_lidar_frame,
    lidar_vec_to_pixel,
)


class LidarCalibCheck(Node):
    def __init__(self):
        super().__init__('lidar_calib_check')

        self.declare_parameter('drone_model_name', 'iris_lidar')
        self.declare_parameter('pad_model_name', 'landing_pad')
        self.declare_parameter('pad_size', 1.0)
        self.declare_parameter('horizontal_samples', 360)
        self.declare_parameter('vertical_samples', 64)
        self.declare_parameter('vertical_min_deg', -50.0)
        self.declare_parameter('vertical_max_deg', 50.0)

        self.drone_name = self.get_parameter('drone_model_name').value
        self.pad_name = self.get_parameter('pad_model_name').value
        self.pad_size = float(self.get_parameter('pad_size').value)
        self.W = int(self.get_parameter('horizontal_samples').value)
        self.H = int(self.get_parameter('vertical_samples').value)
        self.v_min = math.radians(self.get_parameter('vertical_min_deg').value)
        self.v_max = math.radians(self.get_parameter('vertical_max_deg').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        self.latest_depth = None
        self.latest_states = None
        self.create_subscription(Image, '/depth/image', self._depth_cb, qos)
        self.create_subscription(ModelStates, '/gazebo/model_states', self._states_cb, qos)
        self.timer = self.create_timer(2.0, self._check)
        self.get_logger().info('lidar_calib_check 시작 (2초마다 비교 로그 출력)')

    def _depth_cb(self, msg: Image):
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width
        ).copy()
        self.latest_depth = depth

    def _states_cb(self, msg: ModelStates):
        self.latest_states = msg

    def _check(self):
        if self.latest_depth is None or self.latest_states is None:
            self.get_logger().warn('depth 또는 model_states 미수신')
            return

        names = list(self.latest_states.name)
        if self.drone_name not in names or self.pad_name not in names:
            self.get_logger().warn(f'모델 이름 못 찾음: {names}')
            return

        drone_pose = self.latest_states.pose[names.index(self.drone_name)]
        pad_pose = self.latest_states.pose[names.index(self.pad_name)]
        drone_pos = np.array([drone_pose.position.x, drone_pose.position.y, drone_pose.position.z])
        drone_rot = quat_to_rotmat(
            drone_pose.orientation.x, drone_pose.orientation.y,
            drone_pose.orientation.z, drone_pose.orientation.w,
        )
        pad_center = np.array([
            pad_pose.position.x, pad_pose.position.y,
            pad_pose.position.z + self.pad_size * 0.075,
        ])

        lidar_vec = world_point_to_lidar_frame(pad_center, drone_pos, drone_rot)
        calc = lidar_vec_to_pixel(lidar_vec, self.W, self.H, self.v_min, self.v_max)
        if calc is None:
            self.get_logger().warn('계산된 벡터 크기가 0에 가까움')
            return
        calc_col, calc_row, calc_r = calc

        depth = self.latest_depth
        valid = depth > 0.05
        if not np.any(valid):
            self.get_logger().warn('depth 이미지에 유효 포인트 없음 (패드가 시야 밖일 수 있음)')
            return
        masked = np.where(valid, depth, np.inf)
        min_idx = np.unravel_index(np.argmin(masked), masked.shape)
        obs_row, obs_col = int(min_idx[0]), int(min_idx[1])
        obs_r = float(masked[min_idx])

        d_col = abs(calc_col - obs_col)
        d_col = min(d_col, self.W - d_col)
        d_row = abs(calc_row - obs_row)
        d_r = abs(calc_r - obs_r)

        status = 'OK' if (d_col < self.W * 0.05 and d_row < self.H * 0.1) else '!! MISMATCH !!'

        self.get_logger().info(
            f'[{status}] 계산=(col={calc_col:.1f}, row={calc_row:.1f}, r={calc_r:.2f}m)  '
            f'실측(최근접점)=(col={obs_col}, row={obs_row}, r={obs_r:.2f}m)  '
            f'차이=(dcol={d_col:.1f}px, drow={d_row:.1f}px, dr={d_r:.2f}m)'
        )
        if status != 'OK':
            self.get_logger().warn(
                '계산된 패드 방향과 depth 이미지 최근접점 방향이 크게 다릅니다. '
                'dataset_collector_node._LIDAR_T/_LIDAR_R (base_link->lidar 외부파라미터)가 '
                '실제 SDF 구조와 맞지 않을 가능성이 높습니다. merge_iris_lidar.py의 '
                'lidar_link/sensor pose 계층을 다시 확인하세요.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LidarCalibCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
