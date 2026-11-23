#!/usr/bin/env python3
"""
LiDAR 정밀착륙 제어 노드 (회피 방식 재설계 버전).

기존: 회피 벡터를 매 프레임 속도 명령에 더하는 방식(인공 포텐셜 필드).
  -> 문제1: 목표 근처에서 인력이 이기면 장애물에 그대로 박음
  -> 문제2: 인력/척력이 비슷해지면 앞뒤로 진동(limit cycle)
  두 문제 모두 파라미터 튜닝으로 해결 불가한 구조적 한계였음.

변경: 회피를 별도 상태(AVOID)로 분리.
  - 장애물이 위험 수준으로 감지되면 AVOID 진입
  - 진입 시점에 좌/우 중 한 방향을 정해서 "고정"(commit)
  - 최소 유지시간 동안은 판단을 뒤집지 않음  -> 진동 원천 차단
  - 위험이 확실히 해소될 때까지(히스테리시스) 옆으로 빠져나감 -> 박는 것 방지
  - 벗어나면 APPROACH로 복귀

상태머신: SEARCH -> APPROACH -> (AVOID) -> DESCEND -> LAND -> DISARM
"""

import math
import re
import shutil
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped, Vector3Stamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
NAV_STATE_OFFBOARD = 14
ARMING_STATE_ARMED = 2

STATE_SEARCH = 'SEARCH'
STATE_APPROACH = 'APPROACH'
STATE_AVOID = 'AVOID'
STATE_DESCEND = 'DESCEND'
STATE_LAND = 'LAND'

STATUS_REFRESH_HZ = 10
ANSI_ESCAPE_RE = re.compile(r'\x1b\[[0-9;]*m')


def ramp_toward(current, target, max_delta):
    diff = target - current
    if diff > max_delta:
        return current + max_delta
    if diff < -max_delta:
        return current - max_delta
    return target


class LandingControlNode(Node):
    def __init__(self):
        super().__init__('landing_control_node')

        p = self.declare_parameter
        # 제어 게인
        p('kp_xy', 0.8)
        p('kp_xy_land', 0.25)
        p('kd_xy', 0.6)
        p('kd_z', 0.4)
        p('max_xy_speed', 1.0)
        p('max_xy_accel', 2.0)
        p('max_z_accel', 0.6)
        # 착륙 단계
        p('descend_speed', 0.15)
        p('land_final_speed', 0.12)
        p('land_altitude', 0.5)
        p('disarm_altitude', 0.05)
        p('center_tolerance', 0.15)
        p('approach_altitude', 2.5)
        p('land_stuck_timeout', 3.0)
        p('target_lost_timeout', 1.0)
        # 회피 (AVOID 상태)
        p('avoid_enter_threshold', 0.25)   # 회피벡터 크기가 이 이상이면 AVOID 진입
        p('avoid_exit_threshold', 0.10)    # 이 이하로 떨어져야 AVOID 해제(히스테리시스)
        p('avoid_min_hold_sec', 1.5)       # 진입 후 최소 이만큼은 방향 유지(진동 차단)
        p('avoid_max_hold_sec', 8.0)       # 이 이상 못 벗어나면 강제 해제(무한루프 방지)
        p('avoid_side_speed', 0.8)         # 옆으로 빠져나가는 속도
        p('avoid_push_gain', 0.5)          # 장애물 반대방향 성분을 얼마나 섞을지
        p('obstacle_detect_threshold', 0.05)
        p('avoidance_lost_timeout', 0.5)
        # target 좌표 타당성 검사 (절대 기준 - 자기 상태와 비교하지 않음)
        p('target_filter_alpha', 0.3)
        p('target_z_up_margin', 1.0)
        p('target_z_down_margin', 0.5)
        p('target_max_range', 15.0)
        # 토픽
        p('local_pos_topic', '/fmu/out/vehicle_local_position_v1')
        p('vehicle_status_topic', '/fmu/out/vehicle_status_v4')
        p('target_topic', '/landing_target')
        p('avoidance_topic', '/avoidance_velocity')

        g = lambda n: self.get_parameter(n).value
        self.kp_xy = float(g('kp_xy'))
        self.kp_xy_land = float(g('kp_xy_land'))
        self.kd_xy = float(g('kd_xy'))
        self.kd_z = float(g('kd_z'))
        self.max_xy_speed = float(g('max_xy_speed'))
        self.max_xy_accel = float(g('max_xy_accel'))
        self.max_z_accel = float(g('max_z_accel'))
        self.descend_speed = float(g('descend_speed'))
        self.land_final_speed = float(g('land_final_speed'))
        self.land_altitude = float(g('land_altitude'))
        self.disarm_altitude = float(g('disarm_altitude'))
        self.center_tolerance = float(g('center_tolerance'))
        self.approach_altitude = float(g('approach_altitude'))
        self.land_stuck_timeout = float(g('land_stuck_timeout'))
        self.target_lost_timeout = float(g('target_lost_timeout'))
        self.avoid_enter_threshold = float(g('avoid_enter_threshold'))
        self.avoid_exit_threshold = float(g('avoid_exit_threshold'))
        self.avoid_min_hold_sec = float(g('avoid_min_hold_sec'))
        self.avoid_max_hold_sec = float(g('avoid_max_hold_sec'))
        self.avoid_side_speed = float(g('avoid_side_speed'))
        self.avoid_push_gain = float(g('avoid_push_gain'))
        self.obstacle_detect_threshold = float(g('obstacle_detect_threshold'))
        self.avoidance_lost_timeout = float(g('avoidance_lost_timeout'))
        self.target_filter_alpha = float(g('target_filter_alpha'))
        self.target_z_up_margin = float(g('target_z_up_margin'))
        self.target_z_down_margin = float(g('target_z_down_margin'))
        self.target_max_range = float(g('target_max_range'))

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        self.create_subscription(VehicleLocalPosition, g('local_pos_topic'),
                                 self.local_pos_cb, PX4_QOS)
        self.create_subscription(VehicleStatus, g('vehicle_status_topic'),
                                 self.status_cb, PX4_QOS)
        self.create_subscription(PointStamped, g('target_topic'), self.target_cb, 10)
        self.create_subscription(Vector3Stamped, g('avoidance_topic'),
                                 self.avoidance_cb, 10)

        # 상태
        self.drone_x = self.drone_y = self.drone_z = 0.0
        self.drone_vx = self.drone_vy = self.drone_vz = 0.0
        self.home_z = None
        self.nav_state = self.arming_state = None
        self.have_local_pos = False

        self.target_x = self.target_y = self.target_z = None
        self.last_target_time = None

        self.avoid_x = self.avoid_y = self.avoid_z = 0.0
        self.last_avoid_time = None

        self.vx_cmd = self.vy_cmd = self.vz_cmd = 0.0

        self.state = STATE_SEARCH
        self.prev_state = STATE_SEARCH
        self._land_stuck_since = None
        self.disarm_sent = False
        self._last_status_time = None

        # AVOID 전용 상태(방향 커밋)
        self._avoid_since = None
        self._avoid_dir_x = 0.0
        self._avoid_dir_y = 0.0

        self.tick_count = 0
        self.timer = self.create_timer(0.05, self.timer_cb)
        self.get_logger().info('LandingControlNode 시작 (AVOID 상태 방식)')

    # ---------- 콜백 ----------
    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.drone_x, self.drone_y, self.drone_z = msg.x, msg.y, msg.z
        self.drone_vx, self.drone_vy, self.drone_vz = msg.vx, msg.vy, msg.vz
        if self.home_z is None:
            self.home_z = msg.z
        self.have_local_pos = True

    def status_cb(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def target_cb(self, msg: PointStamped):
        nx, ny, nz = msg.point.x, msg.point.y, msg.point.z
        # 절대 타당성 검사(자기 필터 상태가 아니라 외부 기준과 비교 -> 자기 교정 가능)
        if self.home_z is not None:
            z_min = self.home_z - self.target_z_up_margin
            z_max = self.home_z + self.target_z_down_margin
            if not (z_min <= nz <= z_max):
                return
        if self.have_local_pos:
            if math.hypot(nx - self.drone_x, ny - self.drone_y) > self.target_max_range:
                return
        if self.target_x is None:
            self.target_x, self.target_y, self.target_z = nx, ny, nz
        else:
            a = self.target_filter_alpha
            self.target_x = a * nx + (1 - a) * self.target_x
            self.target_y = a * ny + (1 - a) * self.target_y
            self.target_z = a * nz + (1 - a) * self.target_z
        self.last_target_time = self.get_clock().now()

    def avoidance_cb(self, msg: Vector3Stamped):
        self.avoid_x, self.avoid_y, self.avoid_z = msg.vector.x, msg.vector.y, msg.vector.z
        self.last_avoid_time = self.get_clock().now()

    # ---------- 유틸 ----------
    @property
    def altitude_agl(self):
        if self.target_z is not None:
            return self.target_z - self.drone_z
        if self.home_z is None:
            return 0.0
        return self.home_z - self.drone_z

    def target_is_fresh(self):
        if self.last_target_time is None:
            return False
        return (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9 \
            < self.target_lost_timeout

    def avoid_magnitude(self):
        """현재 유효한 회피벡터의 크기(오래된 값이면 0)."""
        if self.last_avoid_time is None:
            return 0.0
        age = (self.get_clock().now() - self.last_avoid_time).nanoseconds / 1e9
        if age > self.avoidance_lost_timeout:
            return 0.0
        return math.sqrt(self.avoid_x ** 2 + self.avoid_y ** 2 + self.avoid_z ** 2)

    def obstacle_detected(self):
        return self.avoid_magnitude() > self.obstacle_detect_threshold

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        m = VehicleCommand()
        m.command, m.param1, m.param2 = command, param1, param2
        m.target_system = m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(m)

    def ensure_offboard_and_armed(self):
        is_off = self.nav_state == NAV_STATE_OFFBOARD
        is_arm = self.arming_state == ARMING_STATE_ARMED
        if not is_off:
            self.send_vehicle_command(VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif not is_arm:
            self.send_vehicle_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        return is_off and is_arm

    def publish_offboard_control_mode(self):
        m = OffboardControlMode()
        m.velocity, m.position = True, False
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(m)

    def publish_velocity_setpoint(self, vx, vy, vz):
        m = TrajectorySetpoint()
        m.velocity = [vx, vy, vz]
        m.position = [float('nan')] * 3
        m.yaw = float('nan')
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(m)

    def log_event(self, text):
        sys.stdout.write('\n')
        sys.stdout.flush()
        self.get_logger().info(text)

    def clamp_xy(self, vx, vy, limit):
        s = math.hypot(vx, vy)
        if s > limit and s > 1e-9:
            k = limit / s
            return vx * k, vy * k
        return vx, vy

    # ---------- AVOID 방향 결정 ----------
    def commit_avoid_direction(self):
        """AVOID 진입 시 한 번만 호출. 목표 방향에 수직인 두 방향 중,
        회피벡터가 가리키는 쪽을 골라서 고정한다."""
        if self.target_x is None:
            bx, by = 1.0, 0.0
        else:
            dx = self.target_x - self.drone_x
            dy = self.target_y - self.drone_y
            n = math.hypot(dx, dy)
            bx, by = (dx / n, dy / n) if n > 1e-6 else (1.0, 0.0)

        # 목표 방향의 좌/우 수직 벡터
        px, py = -by, bx
        # 회피벡터(FRD ~= NED 근사, yaw 제어 안 하므로 heading 고정 가정)와
        # 내적이 양수인 쪽 = 장애물에서 멀어지는 쪽
        if (px * self.avoid_x + py * self.avoid_y) < 0:
            px, py = -px, -py

        self._avoid_dir_x, self._avoid_dir_y = px, py
        self._avoid_since = self.get_clock().now()
        self.log_event(
            f'[AVOID] 장애물 회피 시작 - 방향 고정 ({px:+.2f},{py:+.2f}), '
            f'mag={self.avoid_magnitude():.2f}'
        )

    def avoid_elapsed(self):
        if self._avoid_since is None:
            return 0.0
        return (self.get_clock().now() - self._avoid_since).nanoseconds / 1e9

    # ---------- 상태바 ----------
    def status_bar(self, target_fresh, horiz_err):
        now = self.get_clock().now()
        if self._last_status_time is not None:
            if (now - self._last_status_time).nanoseconds / 1e9 < 1.0 / STATUS_REFRESH_HZ:
                return
        self._last_status_time = now

        tgt = '\033[92mFOUND\033[0m' if target_fresh else '\033[91mLOST \033[0m'
        mag = self.avoid_magnitude()
        obs = f'\033[91m{mag:4.2f}\033[0m' if self.obstacle_detected() \
            else f'\033[92m{mag:4.2f}\033[0m'
        tz = f'{self.target_z:+.2f}' if self.target_z is not None else 'None'

        fields = [
            f'[{self.state:8s}]',
            f'TGT:{tgt}',
            f'OBS:{obs}',
            f'ALT:{self.altitude_agl:+5.2f}',
            f'(tz={tz},dz={self.drone_z:+.2f})',
            f'ERR:{horiz_err:5.2f}',
            f'V:({self.vx_cmd:+.2f},{self.vy_cmd:+.2f},{self.vz_cmd:+.2f})',
        ]
        w = shutil.get_terminal_size((80, 20)).columns
        while True:
            line = ' '.join(fields)
            if len(ANSI_ESCAPE_RE.sub('', line)) <= w - 1 or len(fields) <= 3:
                break
            fields.pop()
        sys.stdout.write('\r\033[2K' + line)
        sys.stdout.flush()

    # ---------- 메인 루프 ----------
    def timer_cb(self):
        self.tick_count += 1
        self.publish_offboard_control_mode()

        if not self.have_local_pos or self.tick_count < 20:
            self.publish_velocity_setpoint(0.0, 0.0, 0.0)
            return

        if not self.ensure_offboard_and_armed() and not self.disarm_sent:
            self.publish_velocity_setpoint(0.0, 0.0, 0.0)
            return

        target_fresh = self.target_is_fresh()
        horiz_err = 0.0
        if self.target_x is not None:
            horiz_err = math.hypot(self.target_x - self.drone_x,
                                   self.target_y - self.drone_y)
        mag = self.avoid_magnitude()

        # ========== 상태 전이 ==========
        if self.state == STATE_LAND:
            pass  # LAND는 되돌아가지 않음

        elif self.state == STATE_AVOID:
            elapsed = self.avoid_elapsed()
            # 최소 유지시간 전에는 절대 해제하지 않음(진동 차단)
            if elapsed >= self.avoid_min_hold_sec:
                if mag < self.avoid_exit_threshold:
                    self.log_event(f'[AVOID] 회피 완료({elapsed:.1f}s) -> 접근 재개')
                    self.state = STATE_APPROACH
                    self._avoid_since = None
                elif elapsed > self.avoid_max_hold_sec:
                    self.log_event(f'[AVOID] 시간초과({elapsed:.1f}s), 강제 해제')
                    self.state = STATE_APPROACH
                    self._avoid_since = None

        elif not target_fresh:
            self.state = STATE_SEARCH

        elif mag > self.avoid_enter_threshold and self.altitude_agl > self.land_altitude:
            # 착륙 최종단계(저고도)에서는 패드/지면 오인 위험이 있어 AVOID 진입 안 함
            self.state = STATE_AVOID
            self.commit_avoid_direction()

        else:
            if self.altitude_agl < self.land_altitude:
                if horiz_err < self.center_tolerance:
                    self.state = STATE_LAND
                    self._land_stuck_since = None
                else:
                    if self._land_stuck_since is None:
                        self._land_stuck_since = self.get_clock().now()
                    stuck = (self.get_clock().now()
                             - self._land_stuck_since).nanoseconds / 1e9
                    self.state = STATE_LAND if stuck > self.land_stuck_timeout \
                        else STATE_APPROACH
            else:
                self._land_stuck_since = None
                self.state = STATE_APPROACH if horiz_err > self.center_tolerance \
                    else STATE_DESCEND

        # ========== 상태별 속도 명령 ==========
        vx = vy = vz = 0.0

        if self.state == STATE_SEARCH:
            vz = -0.3 if self.altitude_agl < self.approach_altitude \
                else -self.kd_z * self.drone_vz

        elif self.state == STATE_AVOID:
            # 고정된 옆방향으로 빠져나가면서, 장애물 반대방향 성분도 약간 섞음.
            # 목표로 끌어당기는 힘은 넣지 않음 -> 인력/척력 상쇄로 인한 정지/진동 없음.
            vx = self._avoid_dir_x * self.avoid_side_speed + self.avoid_x * self.avoid_push_gain
            vy = self._avoid_dir_y * self.avoid_side_speed + self.avoid_y * self.avoid_push_gain
            vx -= self.kd_xy * self.drone_vx
            vy -= self.kd_xy * self.drone_vy
            vx, vy = self.clamp_xy(vx, vy, self.max_xy_speed)
            vz = -self.kd_z * self.drone_vz  # 고도 유지

        elif self.state in (STATE_APPROACH, STATE_DESCEND):
            ex = self.target_x - self.drone_x
            ey = self.target_y - self.drone_y
            vx = self.kp_xy * ex - self.kd_xy * self.drone_vx
            vy = self.kp_xy * ey - self.kd_xy * self.drone_vy
            vx, vy = self.clamp_xy(vx, vy, self.max_xy_speed)
            if self.state == STATE_DESCEND:
                vz = self.descend_speed - self.kd_z * (self.drone_vz - self.descend_speed)
            else:
                vz = -self.kd_z * self.drone_vz

        elif self.state == STATE_LAND:
            ex = self.target_x - self.drone_x
            ey = self.target_y - self.drone_y
            vx = self.kp_xy_land * ex - self.kd_xy * self.drone_vx
            vy = self.kp_xy_land * ey - self.kd_xy * self.drone_vy
            if self.altitude_agl <= self.disarm_altitude:
                vz = 0.0
                if not self.disarm_sent:
                    self.send_vehicle_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                    self.disarm_sent = True
                    self.log_event(f'착륙 완료, disarm (최종 수평오차 {horiz_err:.2f}m)')
            else:
                vz = self.land_final_speed

        # 상태 변화 로그(AVOID 진입/해제는 이미 별도 출력)
        if self.state != self.prev_state and STATE_AVOID not in (self.state, self.prev_state):
            self.log_event(f'상태 전환: {self.prev_state} -> {self.state}')
        self.prev_state = self.state

        # ---- 램프 ----
        dxy = self.max_xy_accel * 0.05
        dz = self.max_z_accel * 0.05
        self.vx_cmd = ramp_toward(self.vx_cmd, vx, dxy)
        self.vy_cmd = ramp_toward(self.vy_cmd, vy, dxy)
        self.vz_cmd = ramp_toward(self.vz_cmd, vz, dz)
        self.publish_velocity_setpoint(self.vx_cmd, self.vy_cmd, self.vz_cmd)

        self.status_bar(target_fresh, horiz_err)


def main(args=None):
    rclpy.init(args=args)
    node = LandingControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write('\n')
        sys.stdout.flush()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
