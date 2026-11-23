#!/usr/bin/env python3
"""
ArUco 마커 기반 PX4 정밀 착륙 노드.

통신 경로: ROS2 <-> MicroXRCEAgent <-> PX4 uxrce_dds_client (MAVROS 아님)
필요 조건:
  - px4_msgs 패키지가 이 워크스페이스에 빌드되어 있어야 함 (PX4 버전과 msg 정의가 맞아야 함)
  - MicroXRCEAgent udp4 -p 8888 가 떠 있어야 함
  - PX4가 이 포트로 uxrce_dds_client를 이미 실행 중이어야 함 (부팅 로그에서 확인됨)

좌표계 가정 (반드시 실제 환경에 맞춰 검증할 것):
  - 카메라는 완전 하향(pitch 90도)으로 장착되어 있다고 가정 (기체에 고정, 짐벌 없음)
  - ArUco 카메라 좌표계: x=오른쪽, y=아래, z=카메라 정면(마커까지의 거리)
  - 이를 body frame(x=전방, y=우측)으로 매핑: body_x = -camera_y, body_y = camera_x
    (마커가 카메라 화면에서 위(-y)에 있으면 드론이 전방으로 가야 마커에 접근)
  - yaw=0 고정 가정은 더 이상 쓰지 않음. 대신 "추격(pursuit)" 방식으로 제어:
    body frame에서 마커가 있는 방향(bearing)을 구해 그 방향으로 기체를 yaw 회전시키고,
    정렬된 만큼 그 방향(현재 heading)으로 전진 속도를 낸다. 즉 몸체가 마커 쪽을 바라보도록
    돌면서 쫓아가는 형태이며, VehicleLocalPosition의 heading(현재 yaw)을 기준으로 삼는다.

터미널 표시:
  - 매 tick마다 로그를 새 줄로 찍으면 터미널이 도배되므로, 상태(state/마커/고도/오차/
    속도/yaw)를 한 줄에 모아 \\r(캐리지리턴)로 계속 덮어쓴다 (status_bar()).
  - 실제 갱신은 STATUS_REFRESH_HZ로 스로틀링해서 눈으로 읽을 수 있는 속도로만 찍는다.
  - 마커 확인/유실 등 의미 있는 이벤트는 같은 줄에 메시지로 얹어서 보여준다.

OFFBOARD/ARM 진입:
  - "명령을 한 번 보냈다"와 "실제로 그 상태가 됐다"는 다르다. DO_SET_MODE나 ARM 명령이
    타이밍(예: 이미 비행 중에 스크립트를 재시작하는 경우) 때문에 거부될 수 있는데,
    예전엔 딱 한 번만 보내고 끝이라 거부되면 그대로 원래 모드(Hold/Position 등)에
    머물러버렸다 -> 속도 명령을 아무리 보내도 무시되고 제자리 고도만 유지되는 버그.
    그래서 VehicleStatus로 실제 nav_state/arming_state가 OFFBOARD+ARMED로 확인될
    때까지 주기적으로 계속 재시도한다(확인되면 재시도 중단).
"""

import math
import traceback
import sys
import shutil
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from aruco_opencv_msgs.msg import ArucoDetection
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)


# PX4 uXRCE-DDS 기본 QoS: BEST_EFFORT + VOLATILE + KEEP_LAST
# 이걸 안 맞추면 subscribe/publish가 에러 없이 그냥 안 붙는다.
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# 상태 머신
STATE_SEARCH = "SEARCH"
STATE_APPROACH = "APPROACH"
STATE_DESCEND = "DESCEND"
STATE_LAND = "LAND"

# 튜닝 파라미터
KP_XY = 1.2               # 수평 위치 오차 -> 속도 게인
MAX_XY_SPEED = 1.2        # m/s, 수평 속도 상한
MAX_XY_ACCEL = 1.6        # m/s^2, 속도 변화율 제한
MAX_YAW_RATE = 1.3        # rad/s, 추격(yaw pursuit) 회전 속도 제한
DESCEND_SPEED = 0.2       # m/s, 하강 속도 (지면 근접 안전 때문에 그대로 유지)
CENTER_TOLERANCE = 0.15   # m, 이 안에 들어오면 "정렬됨"으로 판단
LAND_ALTITUDE = 0.35       # m, 이 고도 이하에서 실제 착륙 명령
DISARM_ALTITUDE = 0.05     # m, 이 고도 이하로 내려오면 NAV_LAND와 별개로 강제 disarm
MARKER_LOST_TIMEOUT = 1.0  # 초, 이 시간 넘게 마커 안 보이면 정지/유지
TIMER_PERIOD = 0.02        # 초, 아래 create_timer 주기와 반드시 일치시킬 것 (50Hz)

MARKER_FILTER_ALPHA = 0.3  # 마커 좌표 EMA 필터 계수 (0~1, 작을수록 더 부드럽지만 반응 느려짐)
YAW_DEADBAND = 0.08        # m, 이 오차 이내면 bearing 재계산 없이 yaw 유지 (원점 근처 atan2 노이즈 증폭 방지)

# OFFBOARD/ARM 확인 전까지 재시도하는 주기 (너무 자주 보내면 커맨드 버스에 부담)
MODE_RETRY_PERIOD_SEC = 0.5
MODE_RETRY_EVERY_N_TICKS = max(1, round(MODE_RETRY_PERIOD_SEC / TIMER_PERIOD))

# 터미널 상태줄 갱신 주기 (너무 빠르면 눈으로 못 읽고 깜빡여서 보임)
STATUS_REFRESH_HZ = 10
STATUS_REFRESH_EVERY_N_TICKS = max(1, round((1.0 / TIMER_PERIOD) / STATUS_REFRESH_HZ))

# ANSI 컬러
COLOR_GREEN = "\033[92m"
COLOR_RED = "\033[91m"
COLOR_RESET = "\033[0m"

# status_bar()에서 터미널 폭 계산 시 색상 코드는 빼고 세기 위한 정규식
_ANSI_RE = re.compile(r"\033\[[0-9;]*m")


class PrecisionLanding(Node):

    def __init__(self):
        super().__init__("precision_landing")

        # 마커 관련 상태
        self.marker_visible = False
        self.marker_confirmed = False  # 타임아웃 기준 "지금 마커 잡혀있음" 상태 (로그 토글용)
        self.marker_ever_seen = False  # 실제로 한 번이라도 마커를 본 적 있는지 (초기 오탐 FOUND 방지)
        self.last_marker_time = self.get_clock().now()
        self.marker_x = 0.0  # camera frame
        self.marker_y = 0.0
        self.marker_z = 0.0  # 마커까지의 거리(대략적인 고도 추정에도 참고 가능)

        # 속도 명령 (NED)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # PX4 상태
        self.arming_state = None
        self.nav_state = None
        self.current_altitude = 0.0  # NED z, 아래가 양수이므로 고도 = -z
        self.current_heading = 0.0   # PX4 heading(rad), local_position_callback에서 갱신
        self.local_position_received = False

        # yaw 추격(pursuit) 제어 상태
        self.yaw_cmd = 0.0  # 지금 명령 중인 목표 yaw (ramp로 서서히 수렴시킴)
        self._heading_warned = False  # heading 필드 없을 때 경고 로그 1회만 찍기 위한 플래그

        # OFFBOARD/ARM 진입 재시도 카운터
        # (예전엔 arm_sent/offboard_sent 플래그로 딱 한 번만 보내고 끝이었는데,
        #  그 한 번이 거부되면 영원히 재시도를 안 해서 "이미 비행 중에 재실행하면
        #  OFFBOARD 진입 실패 -> 이륙 고도에서 계속 hold"하는 버그가 있었음)
        self._mode_retry_tick = 0
        self.offboard_setpoint_counter = 0

        self.land_cmd_sent = False
        self.disarm_sent = False  # 0.05m 이하로 내려왔을 때 강제 disarm 보냈는지

        # 상태줄 갱신 스로틀용 카운터
        self._status_tick = 0

        self.state = STATE_SEARCH

        # ---- Subscribers ----
        self.aruco_sub = self.create_subscription(
            ArucoDetection,
            "/aruco_detections",
            self.aruco_callback,
            10,  # 이건 일반 ROS2 토픽이라 기본 QoS로 충분
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.status_callback,
            PX4_QOS,
        )

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.local_position_callback,
            PX4_QOS,
        )

        # ---- Publishers ----
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            PX4_QOS,
        )

        self.traj_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            PX4_QOS,
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            PX4_QOS,
        )

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)  # 50Hz

        self.get_logger().info("Precision Landing Node 시작 (uXRCE-DDS)")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def aruco_callback(self, msg):
        if len(msg.markers) == 0:
            self.marker_visible = False
            return

        marker = msg.markers[0]
        raw_x = marker.pose.position.x
        raw_y = marker.pose.position.y
        raw_z = marker.pose.position.z

        if self.marker_ever_seen:
            # 저역통과 필터(EMA): ArUco 포즈 추정의 프레임 단위 떨림을 완화.
            # 특히 마커가 화면 중심 근처일수록 atan2(bearing 계산)가 작은 노이즈에도
            # 각도를 크게 증폭시키기 때문에, 원본 좌표 단계에서부터 부드럽게 만들어준다.
            a = MARKER_FILTER_ALPHA
            self.marker_x = a * raw_x + (1 - a) * self.marker_x
            self.marker_y = a * raw_y + (1 - a) * self.marker_y
            self.marker_z = a * raw_z + (1 - a) * self.marker_z
        else:
            # 첫 감지는 필터링 없이 그대로 반영(초기값이 0.0이라 EMA 적용하면 처음에
            # 실제 위치까지 느리게 수렴하게 됨)
            self.marker_x = raw_x
            self.marker_y = raw_y
            self.marker_z = raw_z

        self.marker_visible = True
        self.marker_ever_seen = True
        self.last_marker_time = self.get_clock().now()

    def status_callback(self, msg):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state

    def local_position_callback(self, msg):
        # PX4 local position은 NED: z가 아래로 양수. 고도는 -z.
        self.current_altitude = -msg.z

        # msg.heading이 이 px4_msgs 빌드에 없을 수도 있으므로 방어적으로 접근.
        # 없으면 노드를 죽이는 대신 0.0으로 두고 1회만 경고 로그를 남긴다.
        heading = getattr(msg, "heading", None)
        if heading is None:
            if not self._heading_warned:
                self.get_logger().error(
                    "VehicleLocalPosition에 'heading' 필드가 없음 -> "
                    "yaw 추격이 heading=0.0으로 고정됨. px4_msgs 버전 확인 필요 "
                    "(예: `ros2 interface show px4_msgs/msg/VehicleLocalPosition`)"
                )
                self._heading_warned = True
            heading = 0.0
        self.current_heading = heading

        if not self.local_position_received:
            # 첫 수신 시점에는 yaw_cmd를 실제 현재 heading으로 맞춰서
            # 이후 ramp가 0.0이 아니라 실제 자세에서 시작하도록 한다.
            self.yaw_cmd = heading
        self.local_position_received = True
        # 예전엔 여기서 매 tick "arming=..., nav=..." 를 get_logger().info로 찍었는데
        # 이게 터미널 도배의 주범이었음. 이제 이 정보는 status_bar() 한 줄에 통합됨.

    def log_event(self, text, color=COLOR_RESET):
        """
        상태 이벤트(마커 확인/유실, 착륙 등)를 상태줄과 완전히 분리된 새 줄에 출력한다.
        한글은 터미널에서 2칸 폭을 차지해서 고정폭 status_bar()에 같이 넣으면
        폭 계산이 틀어져 줄바꿈(wrap)이 생기고 \\033[2K로도 안 지워지는 문제가 있었음.
        그래서 이벤트는 반드시 자기 줄을 새로 만들어서 찍는다.
        """
        sys.stdout.write("\n" + color + text + COLOR_RESET + "\n")
        sys.stdout.flush()

    def status_bar(self):
        """
        state/마커/OFFBOARD·ARM/고도/오차/속도/yaw를 담은, 폭이 고정된 ASCII 전용 한 줄.
        한글/가변 메시지는 절대 여기 안 섞는다(폭 계산 문제로 줄바꿈 유발 -> log_event 참고).

        터미널 폭을 넘기면 wrap이 생기고 \\033[2K로는 현재 줄만 지워져서 잔여물이 쌓이는
        문제가 있었음(처음엔 한글 때문에, 이번엔 필드 추가로 순수 길이 초과 때문에 재발함).
        그래서 실제 터미널 폭을 재서, 넘칠 것 같으면 덜 중요한 필드(YAW -> V)부터 순서대로
        빼서 항상 한 줄 안에 들어가도록 만든다.
        """
        err = self.horizontal_error() if self.marker_visible else 0.0
        lost = self.marker_is_stale()
        marker_color = COLOR_RED if lost else COLOR_GREEN
        marker_text = "LOST" if lost else "FOUND"

        offboard = (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        armed = (self.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        px4_color = COLOR_GREEN if (offboard and armed) else COLOR_RED
        # 배지를 2글자로 압축: O/- (offboard 여부) + A/- (armed 여부)
        px4_text = ("O" if offboard else "-") + ("A" if armed else "-")

        # 우선순위 높은 것부터: state/marker/offb·arm/alt/err는 항상 표시,
        # v/yaw는 폭이 부족하면 뒤에서부터 뺀다.
        core = (
            f"[{self.state:^8}] "
            f"[{marker_color}{marker_text:^5}{COLOR_RESET}] "
            f"[{px4_color}{px4_text:^2}{COLOR_RESET}] "
            f"ALT:{self.current_altitude:5.2f}m "
            f"ERR:{err:4.2f}m"
        )
        v_part = f" V:({self.vx:+4.2f},{self.vy:+4.2f},{self.vz:+4.2f})"
        yaw_part = f" YAW:{math.degrees(self.yaw_cmd):6.1f}°"

        columns = shutil.get_terminal_size(fallback=(100, 24)).columns
        # ANSI 색상 코드는 화면에 안 찍히니까 폭 계산에서 빼고 순수 글자 수만 비교
        visible_len = lambda s: len(_ANSI_RE.sub("", s))

        parts = [core, v_part, yaw_part]
        while len(parts) > 1 and visible_len("".join(parts)) > columns - 1:
            parts.pop()  # 뒤(덜 중요한 필드)부터 제거

        line = "\r" + "".join(parts)

        sys.stdout.write("\033[2K\r" + line)
        sys.stdout.flush()

    # ------------------------------------------------------------------
    # Command helpers
    # ------------------------------------------------------------------

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self, vx, vy, vz, yaw=float("nan")):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float("nan"), float("nan"), float("nan")]
        msg.velocity = [vx, vy, vz]
        msg.yaw = yaw
        self.traj_pub.publish(msg)

    # ------------------------------------------------------------------
    # 좌표 변환: ArUco 카메라 좌표 -> NED 속도 명령
    # ------------------------------------------------------------------

    def compute_pursuit_command(self):
        """
        카메라 좌표(x=오른쪽, y=아래)를 body frame(x=전방, y=우측)으로 매핑한 뒤,
        그 방향(bearing)을 향해 기체를 yaw로 돌리면서(추격) 정렬된 만큼 전진 속도를 낸다.

        body_x = -marker_y  (마커가 화면 위쪽이면 전방)
        body_y = marker_x   (마커가 화면 오른쪽이면 우측)
        bearing = atan2(body_y, body_x)  # 기체 정면 기준 마커 방향 각도
        target_yaw = 현재 heading + bearing  -> 이 방향으로 기체를 돌린다

        속도는 별도 vx/vy 대신, "정렬된 방향으로의 전진 속도" 하나로 통일한다.
        """
        body_x = -self.marker_y
        body_y = self.marker_x

        if self.horizontal_error() < YAW_DEADBAND:
            # 마커가 화면 중심 근처면 atan2(bearing)가 아주 작은 노이즈에도
            # 각도를 크게 증폭시켜서 yaw가 떨리게 된다. 이 정도로 이미 정렬됐으면
            # 굳이 더 돌 필요도 없으니, 방향 계산을 멈추고 지금 yaw를 그대로 유지한다.
            target_yaw = self.yaw_cmd
        else:
            bearing = math.atan2(body_y, body_x)
            target_yaw = self.current_heading + bearing

        speed = min(MAX_XY_SPEED, KP_XY * self.horizontal_error())
        return target_yaw, speed

    @staticmethod
    def ramp_angle_toward(current, target, dt, max_rate=MAX_YAW_RATE):
        """각도용 ramp_toward. -pi~pi 랩어라운드를 고려해 최단 방향으로 회전시킨다."""
        diff = (target - current + math.pi) % (2 * math.pi) - math.pi
        max_delta = max_rate * dt
        if diff > max_delta:
            diff = max_delta
        elif diff < -max_delta:
            diff = -max_delta
        return current + diff

    def horizontal_error(self):
        return (self.marker_x ** 2 + self.marker_y ** 2) ** 0.5

    @staticmethod
    def ramp_toward(current, target, dt, max_accel=MAX_XY_ACCEL):
        """
        current 속도를 target 속도 쪽으로 max_accel(m/s^2) 한도 내에서만 이동시킨다.
        """
        max_delta = max_accel * dt
        delta = target - current
        if delta > max_delta:
            delta = max_delta
        elif delta < -max_delta:
            delta = -max_delta
        return current + delta

    # ------------------------------------------------------------------
    # 상태 머신
    # ------------------------------------------------------------------

    def marker_is_stale(self):
        if not self.marker_ever_seen:
            return True  # 한 번도 못 봤으면 무조건 "유실" 취급 (초기 오탐 FOUND 방지)
        elapsed = (self.get_clock().now() - self.last_marker_time).nanoseconds / 1e9
        return elapsed > MARKER_LOST_TIMEOUT

    def log_marker_presence(self):
        stale = self.marker_is_stale()

        if not stale and not self.marker_confirmed:
            self.log_event("마커 확인", COLOR_GREEN)
            self.marker_confirmed = True

        elif stale and self.marker_confirmed:
            if self.state != STATE_LAND:
                # LAND 상태에서는 착지 직전 카메라가 너무 가까워져 마커를 놓치는 게
                # 정상적인 현상이라 굳이 "유실" 이벤트로 알릴 필요가 없음
                self.log_event("마커 유실", COLOR_RED)
            self.marker_confirmed = False

    def update_state_machine(self):
        self.log_marker_presence()

        if self.marker_is_stale():
            if self.state == STATE_LAND:
                # 이미 착륙 명령을 보낸 상태면 마커 유실과 무관하게 그대로 둔다
                return
            # 마커를 놓쳤으면 안전하게 정지(제자리 유지), 하강 중단
            # (급정지 대신 같은 가속도 한도로 부드럽게 0까지 감속)
            self.vx = self.ramp_toward(self.vx, 0.0, TIMER_PERIOD)
            self.vy = self.ramp_toward(self.vy, 0.0, TIMER_PERIOD)
            self.vz = self.ramp_toward(self.vz, 0.0, TIMER_PERIOD)

            self.state = STATE_SEARCH
            return

        err = self.horizontal_error()

        if self.state == STATE_SEARCH:
            self.state = STATE_APPROACH

        if self.state == STATE_APPROACH:
            target_yaw, target_speed = self.compute_pursuit_command()
            self.yaw_cmd = self.ramp_angle_toward(self.yaw_cmd, target_yaw, TIMER_PERIOD)
            forward_speed = self.ramp_toward(
                math.hypot(self.vx, self.vy), target_speed, TIMER_PERIOD
            )
            self.vx = forward_speed * math.cos(self.yaw_cmd)
            self.vy = forward_speed * math.sin(self.yaw_cmd)
            self.vz = 0.0
            if err < CENTER_TOLERANCE:
                self.state = STATE_DESCEND

        elif self.state == STATE_DESCEND:
            target_yaw, target_speed = self.compute_pursuit_command()
            self.yaw_cmd = self.ramp_angle_toward(self.yaw_cmd, target_yaw, TIMER_PERIOD)
            forward_speed = self.ramp_toward(
                math.hypot(self.vx, self.vy), target_speed, TIMER_PERIOD
            )
            self.vx = forward_speed * math.cos(self.yaw_cmd)
            self.vy = forward_speed * math.sin(self.yaw_cmd)
            self.vz = DESCEND_SPEED  # NED라 아래로 이동 = 양수

            if err > CENTER_TOLERANCE * 2:
                # 하강 중 정렬이 심하게 틀어지면 다시 정렬 우선
                self.state = STATE_APPROACH

            if (
                self.local_position_received
                and self.current_altitude < LAND_ALTITUDE
                and err < CENTER_TOLERANCE
            ):
                self.log_event("LAND 조건 만족", COLOR_GREEN)
                self.state = STATE_LAND

        elif self.state == STATE_LAND:
            self.vx = 0.0
            self.vy = 0.0
            self.vz = 0.0
            if not self.land_cmd_sent:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.log_event("착륙 명령 전송", COLOR_GREEN)
                self.land_cmd_sent = True

            # NAV_LAND는 PX4 착륙 감지기가 확인해야 자동 disarm하는데, 감지가
            # 늦거나 안 걸리는 경우를 대비한 안전망: 이 고도까지 내려오면 강제 disarm
            if (
                not self.disarm_sent
                and self.local_position_received
                and self.current_altitude <= DISARM_ALTITUDE
            ):
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=0.0,  # 0.0 = disarm
                )
                self.log_event("고도 0.05m 이하 -> 강제 disarm", COLOR_GREEN)
                self.disarm_sent = True

    # ------------------------------------------------------------------
    # OFFBOARD/ARM 진입 (확인될 때까지 재시도)
    # ------------------------------------------------------------------

    def ensure_offboard_and_armed(self):
        """
        예전엔 arm_sent/offboard_sent 플래그로 딱 한 번씩만 보내고 끝났는데,
        그 시도가 거부되면(특히 이미 비행 중인 상태에서 스크립트를 재시작하는 경우)
        영원히 원래 모드에 머물러버리는 버그가 있었다. 그래서 VehicleStatus로
        실제 상태가 확인될 때까지 주기적으로 계속 재시도한다.
        """
        offboard = (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        armed = (self.arming_state == VehicleStatus.ARMING_STATE_ARMED)

        if offboard and armed:
            return  # 이미 둘 다 확인됨 -> 더 이상 보낼 필요 없음

        self._mode_retry_tick += 1
        if self._mode_retry_tick < MODE_RETRY_EVERY_N_TICKS:
            return
        self._mode_retry_tick = 0

        if not offboard:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0,
            )
        if not armed:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
            )

    # ------------------------------------------------------------------
    # Main timer loop
    # ------------------------------------------------------------------

    def timer_callback(self):
        try:
            self.publish_offboard_control_mode()

            self.update_state_machine()
            self.publish_trajectory_setpoint(self.vx, self.vy, self.vz, yaw=self.yaw_cmd)

            # 상태줄은 매 tick(50Hz)이 아니라 STATUS_REFRESH_HZ로 스로틀링해서 갱신
            # (너무 빠르면 눈에 깜빡임만 보이고 오히려 안 읽힘)
            self._status_tick += 1
            if self._status_tick >= STATUS_REFRESH_EVERY_N_TICKS:
                self._status_tick = 0
                self.status_bar()

            if self.offboard_setpoint_counter < 10:
                self.offboard_setpoint_counter += 1
                return

            # OFFBOARD 진입 + ARM: 확인될 때까지 계속 재시도 (한 번 보내고 끝내지 않음)
            self.ensure_offboard_and_armed()
        except Exception:
            # 상태줄 위에 traceback이 겹치지 않도록 줄바꿈 한 번 하고 에러 로그 출력
            sys.stdout.write("\n")
            sys.stdout.flush()
            self.get_logger().error(
                "timer_callback에서 예외 발생 (노드는 계속 실행됨):\n"
                + traceback.format_exc()
            )


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLanding()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 상태줄(캐리지리턴 상태) 위에 셸 프롬프트가 겹쳐 보이지 않도록 줄바꿈 한 번
        sys.stdout.write("\n")
        sys.stdout.flush()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
