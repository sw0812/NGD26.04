#!/usr/bin/env python3
"""격자선(라인트레이싱) 기반 수평 위치 추정 -> PX4 외부비전(EV)으로 전송.

원리 (이산-연속 융합):
  1) grid_line_follow.GridLineFollower로 하향 카메라에서 격자선(Hough) 검출
  2) 선 위의 두 픽셀을 카메라 광선 -> 기체 자세(q)로 회전 -> 지면(높이 = 라이다
     dist_bottom - 0.05)과 교차시켜 기체 기준 지면 선분(N,E)으로 변환
  3) 남북 방향 선이면 East 좌표 하나, 동서 방향이면 North 좌표 하나를 줌.
     "기체 사전위치(EKF) + 선까지 상대거리"를 가장 가까운 실제 격자선 좌표(3m
     간격, 필드 정의 waypoints.yaml)에 스냅 -> 측정 위치 = 격자선 좌표 - 상대거리
  4) 축별 중앙값을 /fmu/in/vehicle_visual_odometry (NED)로 발행.
     못 본 축은 사전위치 + 큰 분산(=사실상 무시)으로 채움.

PX4가 이 값을 실제로 쓰는지는 EKF2_EV_CTRL에 달림 (0이면 무시 -> 검증용으로 안전).
/line_odom/debug: 검출선 + 측정값 vs EKF 표시, 로그에 2초마다 오차 통계.

좌표: world ENU(x=East,y=North) = PX4 local NED의 (y, x). 버티포트=원점.
"""
import math
import statistics

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleAttitude
from std_msgs.msg import String, Float32
from aruco_opencv_msgs.msg import ArucoDetection
import json

from grid_line_follow import GridLineFollower
from fused_line import Cam, FusedLine, detect_segments

# 격자선 world 좌표 (ENU). mission area 2..26 x 2..17, 3m 간격 (waypoints.yaml)
# 2026-11-10 field_texture.png 직접 측정으로 확인: + 안전구역 테두리 (E -2/30, N -2/21)
X_LINES = [-2.0] + [2.0 + 3.0 * i for i in range(9)] + [30.0]   # East 일정 (남북 방향 선)
Y_LINES = [-2.0] + [2.0 + 3.0 * j for j in range(6)] + [21.0]   # North 일정 (동서 방향 선)
# 버티포트(원점, 반지름 1.5m, 높이 0.7m): 위에 있으면 라이다가 상판까지 거리를 재서
# 바닥 선 투영 높이가 0.7m 틀림 -> run8b(GPS 끔)에서 ~0.6m 편향이 누적돼 옆 격자선에
# 잠기고 기체 이탈. 이 반경 안에선 측정 안 함, 지면점이 이 안에 찍히는 선(상판 테두리/V)도 버림.
VERTIPORT_EXCLUDE_R = 2.5
PAD_LINE_EXCLUDE_R = 1.8
MAX_INNOV = 0.5         # m, EKF 예측과 이보다 차이 나는 측정은 버림(스냅 오류/편향 방지)
SNAP_TOL = 0.7          # m, 사전위치 기준 이보다 멀면 어느 선인지 불확실 -> 버림
AXIS_ANGLE_TOL = math.radians(20)  # 지면 선분이 남북/동서에서 이 이상 틀어지면 버림
CAM_BELOW_LIDAR = 0.05  # m (camera z -0.15, lidar z -0.10)
MEAS_STD = 0.08         # m, 측정 표준편차 (높이에 비례해 키움). 2026-11-13 사용자 결정 0.10 -> 0.08
                        # (30회 스윕: 0.07/0.10 둘 다 6/6, 0.07이 최대오차 작음. 2.85 m에서 EKF로 0.114 m 전달, 하한 0.10 위)
UNMEASURED_VAR = 100.0  # m^2
# 2026-11-12 한 칸 어긋남 감지/복구 (RELOC): 위치를 아는 ArUco(버티포트 ID0 + 미션노드가 등록한 마커)를
# 하향 카메라로 보면 절대위치 계산 -> EKF와 RELOC_MIN 이상 차이가 RELOC_CONFIRM 프레임 연속(서로 RELOC_AGREE 안)이면
# VehicleOdometry.reset_counter+1 로 EKF 수평위치를 그 값으로 리셋 (PX4 ev_control.cpp:54, ev_pos_control.cpp:252).
RELOC_MIN = 1.0         # m, 이보다 작은 차이는 라인 측정이 알아서 잡음 -> 건드리지 않음
RELOC_MAX = 4.5         # m, 1.5칸 넘게 차이나면 ID 오등록/오검출 의심 -> 무시
RELOC_CONFIRM = 3       # 연속 프레임 수
RELOC_AGREE = 0.3       # m, 연속 프레임끼리 일치 범위
RELOC_COOLDOWN = 3.0    # s, 리셋 후 대기
RELOC_MAX_RANGE = 4.0   # m, 마커까지 거리 이보다 멀면 안 씀
RELOC_STD = 0.05        # m, 리셋 위치 표준편차
RELOC_STREAM = 1.0      # s, 리셋 뒤 이 시간 동안 같은 마커 절대위치를 계속 EV로 보냄
                        # (버티포트 근처처럼 EV 융합이 꺼져 있으면 메시지 1개는 무시됨 -> 연속으로 보내야 EKF가 재시작+리셋)
DOWN_CAM_OFFSET = (0.0, 0.0, 0.15)   # body FRD (model.sdf camera_down z -0.15)
MIN_HEIGHT = 0.8        # m, 이보다 낮으면 화면에 선이 거의 없음/부정확

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)


def quat_to_rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def snap(v, grid):
    g = min(grid, key=lambda c: abs(c - v))
    return g if abs(g - v) < SNAP_TOL else None


class LineOdometry(Node):
    def __init__(self):
        super().__init__("line_odometry")
        self.declare_parameter("fx", 927.5)  # 2026-11-14 하향 3.2mm 기본값 (4mm 1159.4) (2.8mm 811.6) (44.6도 1775.0, 16mm 4637.7, 6mm 1739.1) (이전 205.4696)
        self.declare_parameter("cx", 728.0)
        self.declare_parameter("cy", 544.0)
        self.fx = self.get_parameter("fx").value
        # 2026-11-11: 앙상블 통계로 조정 가능하게 파라미터화 (기본값 = 이전 상수)
        self.declare_parameter("meas_std", MEAS_STD)
        self.declare_parameter("max_innov", MAX_INNOV)
        self.meas_std = float(self.get_parameter("meas_std").value)
        self.max_innov = float(self.get_parameter("max_innov").value)
        self.get_logger().info(f"meas_std={self.meas_std} max_innov={self.max_innov}")
        self.cx = self.get_parameter("cx").value
        self.cy = self.get_parameter("cy").value
        self.follower = GridLineFollower(cx=self.cx, cy=self.cy)
        # 2026-11-12: 새 카메라(RPi GS 4mm)에선 blackhat 검출기 0회 -> 기본은 윤곽 쌍 검출기(fused_line, 하향만)
        self.declare_parameter("detector", "fused")
        self.detector = self.get_parameter("detector").value
        self.fl = FusedLine(Cam(self.fx, self.cx, self.cy, math.pi / 2, (0.0, 0.0, 0.15)),
                            Cam(self.fx, self.cx, self.cy, 0.6109, (0.10, 0.0, -0.02)))
        self.get_logger().info(f"detector={self.detector} fx={self.fx}")
        self.bridge = CvBridge()
        self.lp = None
        self.q = None
        self.stats = {"n": 0, "dn": [], "de": []}
        self.why = {}

        img_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, img_qos)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
                                 lambda m: setattr(self, "lp", m), PX4_QOS)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude",
                                 lambda m: setattr(self, "q", list(m.q)), PX4_QOS)
        self.pub = self.create_publisher(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", PX4_QOS)
        # RELOC: 알려진 마커 위치 {id: (E, N)} — ID0 버티포트는 항상 원점
        self.declare_parameter("reloc", True)
        self.reloc = bool(self.get_parameter("reloc").value)
        # 2026-11-14 B: 격자선 각도로 헤딩 측정 -> EV yaw (EKF2_EV_CTRL bit3 필요). 기본 끔.
        self.declare_parameter("ev_yaw", False)
        self.ev_yaw = bool(self.get_parameter("ev_yaw").value)
        self.n_yaw = 0
        self.get_logger().info(f"ev_yaw={self.ev_yaw}")
        # 2026-11-14 출발 보류용: 격자선 기준 헤딩 오차 d [deg] (= EKF yaw - 격자 yaw)
        self.ydev_pub = self.create_publisher(Float32, "/line_odom/yaw_dev", 10)
        self.known = {0: (0.0, 0.0)}
        self.reset_counter = 0
        self._reloc_hist = []
        self._reloc_last = None
        self._reloc_stream = None   # (marker_id, 끝나는 시각)
        self.n_reloc = 0
        self._reloc_chk = []   # (id, dN, dE) 검증용 — 2초마다 요약 로그
        self.create_subscription(String, "/mission/markers", self.on_markers, 10)
        self.create_subscription(ArucoDetection, "/aruco_detections", self.on_aruco, 10)
        self.dbg = self.create_publisher(Image, "/line_odom/debug", 1)
        self.create_timer(2.0, self.report)
        self.declare_parameter("csv", "")
        c = self.get_parameter("csv").value
        self.csv = open(c, "w") if c else None
        if self.csv:
            self.csv.write("t,img_stamp,err_n,err_e,vn,ve,roll,pitch,h,pn,pe\n")

    # 픽셀 -> 기체 기준 지면점 (dN, dE)
    def ground(self, u, v, R, h):
        ray = np.array([-(v - self.cy) / self.fx, (u - self.cx) / self.fx, 1.0])  # body FRD
        r = R @ ray
        if r[2] < 0.2:
            return None
        t = h / r[2]
        return t * r[0], t * r[1]

    def on_image(self, msg):
        if self.lp is None or self.q is None or not self.lp.dist_bottom_valid:
            return
        h = self.lp.dist_bottom - CAM_BELOW_LIDAR
        if h < MIN_HEIGHT:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        R = quat_to_rot(self.q)
        prior_n, prior_e = self.lp.x, self.lp.y
        if math.hypot(prior_n, prior_e) < VERTIPORT_EXCLUDE_R:
            # B: 선 '방향'은 높이와 무관 -> 버티포트 위에서도 헤딩 측정은 함 (위치는 안 함)
            if self.ev_yaw:
                ym = self._yaw_meas(frame)
                if ym is not None:
                    o = VehicleOdometry()
                    o.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                    o.timestamp_sample = o.timestamp
                    o.pose_frame = VehicleOdometry.POSE_FRAME_NED
                    o.position = [float(prior_n), float(prior_e), float(self.lp.z)]
                    o.position_variance = [UNMEASURED_VAR] * 3
                    o.q = [math.cos(ym[0] / 2), 0.0, 0.0, math.sin(ym[0] / 2)]
                    o.orientation_variance = [ym[1]] * 3
                    o.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
                    o.velocity = [float("nan")] * 3
                    o.angular_velocity = [float("nan")] * 3
                    o.velocity_variance = [float("nan")] * 3
                    o.reset_counter = self.reset_counter
                    self.pub.publish(o)
                    self.n_yaw += 1
            return
        meas_n, meas_e, used = [], [], []
        self._why("frames")
        if self.detector == "fused":
            self._fused_meas(frame, prior_n, prior_e, meas_n, meas_e, used)
            lines = []
        else:
            lines = self.follower.lines(frame)
            if not lines:
                self._why("no_lines")
        for rho, th in lines:
            base = np.array([rho * math.cos(th), rho * math.sin(th)])
            d = np.array([-math.sin(th), math.cos(th)])
            foot = base + (np.array([self.cx, self.cy]) - base) @ d * d
            p1 = self.ground(*(foot - 100 * d), R, h)
            p2 = self.ground(*(foot + 100 * d), R, h)
            if p1 is None or p2 is None:
                self._why("no_ground"); continue
            # 선의 지면 위치가 버티포트 근처면(상판 테두리/V 표시일 가능성) 버림
            mid_n, mid_e = prior_n + (p1[0] + p2[0]) / 2, prior_e + (p1[1] + p2[1]) / 2
            if math.hypot(mid_n, mid_e) < PAD_LINE_EXCLUDE_R:
                self._why("pad"); continue
            dn, de = p2[0] - p1[0], p2[1] - p1[1]
            ang = math.atan2(de, dn)  # 0 = 북쪽 방향 선
            a = abs((ang + math.pi / 2) % math.pi - math.pi / 2)  # 남북선과의 각도 0..pi/2
            if a < AXIS_ANGLE_TOL and abs(dn) > 1e-6:
                # 남북 방향 선 -> 기체 North 위치(=0)에서의 East 오프셋
                rel_e = p1[1] + (0 - p1[0]) * de / dn
                g = snap(prior_e + rel_e, X_LINES)
                if g is None:
                    self._why("snapE_fail")
                elif abs(g - rel_e - prior_e) >= self.max_innov:
                    self._why("innovE")
                else:
                    meas_e.append(g - rel_e); used.append((rho, th, "E", g))
            elif abs(a - math.pi / 2) < AXIS_ANGLE_TOL and abs(de) > 1e-6:
                rel_n = p1[0] + (0 - p1[1]) * dn / de
                g = snap(prior_n + rel_n, Y_LINES)
                if g is None:
                    self._why("snapN_fail")
                elif abs(g - rel_n - prior_n) >= self.max_innov:
                    self._why("innovN")
                else:
                    meas_n.append(g - rel_n); used.append((rho, th, "N", g))

        yaw_meas = self._yaw_meas(frame) if self.ev_yaw else None   # (yaw, var) or None
        if meas_n or meas_e or yaw_meas is not None:
            var = (self.meas_std * max(1.0, h / 2.0)) ** 2
            o = VehicleOdometry()
            o.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            o.timestamp_sample = o.timestamp
            o.pose_frame = VehicleOdometry.POSE_FRAME_NED
            n = statistics.median(meas_n) if meas_n else prior_n
            e = statistics.median(meas_e) if meas_e else prior_e
            # PX4 EKF2는 position 3축이 전부 finite여야 샘플을 받음(EKF2.cpp isAllFinite) —
            # z는 EKF 값 그대로 + 큰 분산 (EKF2_EV_CTRL=1이면 수평만 쓰므로 영향 없음)
            o.position = [float(n), float(e), float(self.lp.z)]
            o.position_variance = [var if meas_n else UNMEASURED_VAR,
                                   var if meas_e else UNMEASURED_VAR, UNMEASURED_VAR]
            o.q = [float("nan")] * 4
            o.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            o.velocity = [float("nan")] * 3
            o.angular_velocity = [float("nan")] * 3
            o.orientation_variance = [float("nan")] * 3
            o.velocity_variance = [float("nan")] * 3
            if yaw_meas is not None:   # yaw만 담은 쿼터니언 (EKF는 yaw만 씀)
                yw, yv = yaw_meas
                o.q = [math.cos(yw / 2), 0.0, 0.0, math.sin(yw / 2)]
                o.orientation_variance = [yv, yv, yv]
                self.n_yaw += 1
            o.reset_counter = self.reset_counter
            self.pub.publish(o)
            self.stats["n"] += 1
            if meas_n:
                self.stats["dn"].append(n - prior_n)
            if meas_e:
                self.stats["de"].append(e - prior_e)
            if self.csv:
                w, x, y, z = self.q
                roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
                pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
                st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.csv.write(f"{self.get_clock().now().nanoseconds*1e-9:.3f},{st:.3f},"
                               f"{(n - prior_n) if meas_n else float('nan'):.3f},"
                               f"{(e - prior_e) if meas_e else float('nan'):.3f},"
                               f"{self.lp.vx:.3f},{self.lp.vy:.3f},{roll:.4f},{pitch:.4f},{h:.2f},{prior_n:.2f},{prior_e:.2f}\n")
                self.csv.flush()
        self.publish_debug(frame, used, meas_n, meas_e, prior_n, prior_e, msg.header)

    def on_markers(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        for k, v in d.items():
            k = int(k)
            if k != 0 and k not in self.known:
                self.known[k] = (float(v[0]), float(v[1]))
                self.get_logger().info(f"[RELOC] 기준 마커 추가 ID{k} @ E{v[0]:.0f} N{v[1]:.0f}")

    def on_aruco(self, msg):
        """하향 ArUco로 절대위치 -> EKF와 비교 -> 한 칸 어긋남이면 EKF 위치 리셋."""
        if not self.reloc or self.lp is None or self.q is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        streaming = self._reloc_stream is not None and now < self._reloc_stream[1]
        if not streaming and self._reloc_last is not None and now - self._reloc_last < RELOC_COOLDOWN:
            return
        R = quat_to_rot(self.q)
        for mk in msg.markers:
            if mk.marker_id not in self.known:
                continue
            po = np.array([mk.pose.position.x, mk.pose.position.y, mk.pose.position.z])
            if not np.all(np.isfinite(po)) or np.linalg.norm(po) > RELOC_MAX_RANGE:
                continue
            body = np.array(DOWN_CAM_OFFSET) + np.array([-po[1], po[0], po[2]])   # optical(x右,y下,z광축) -> FRD
            ned = R @ body
            mE, mN = self.known[mk.marker_id]
            abs_n, abs_e = mN - ned[0], mE - ned[1]
            dn, de = abs_n - self.lp.x, abs_e - self.lp.y
            d = math.hypot(dn, de)
            if streaming:
                if mk.marker_id == self._reloc_stream[0]:
                    self._publish_abs(abs_n, abs_e)
                    return
                continue
            self._reloc_chk.append((mk.marker_id, dn, de))
            if d < RELOC_MIN or d > RELOC_MAX:
                self._reloc_hist = []
                continue
            h = self._reloc_hist
            if h and (h[-1][0] != mk.marker_id or math.hypot(h[-1][1] - abs_n, h[-1][2] - abs_e) > RELOC_AGREE):
                h.clear()
            h.append((mk.marker_id, abs_n, abs_e))
            if len(h) < RELOC_CONFIRM:
                return
            n = statistics.median(x[1] for x in h)
            e = statistics.median(x[2] for x in h)
            self.reset_counter = (self.reset_counter + 1) % 256
            self._publish_abs(n, e)
            self._reloc_stream = (mk.marker_id, now + RELOC_STREAM)
            self.n_reloc += 1
            self._reloc_last = now
            self._reloc_hist = []
            self.get_logger().warning(
                f"[RELOC] ID{mk.marker_id} 기준 EKF 위치 리셋: EKF(E{self.lp.y:.2f},N{self.lp.x:.2f}) -> "
                f"(E{e:.2f},N{n:.2f})  차이 dE{e - self.lp.y:+.2f} dN{n - self.lp.x:+.2f} m  (#{self.n_reloc})")
            return

    def _publish_abs(self, n, e):
        """ArUco 절대위치를 EV로 1개 발행 (현재 reset_counter 사용)."""
        o = VehicleOdometry()
        o.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        o.timestamp_sample = o.timestamp
        o.pose_frame = VehicleOdometry.POSE_FRAME_NED
        o.position = [float(n), float(e), float(self.lp.z)]
        o.position_variance = [RELOC_STD ** 2, RELOC_STD ** 2, UNMEASURED_VAR]
        o.q = [float("nan")] * 4
        o.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        o.velocity = [float("nan")] * 3
        o.angular_velocity = [float("nan")] * 3
        o.orientation_variance = [float("nan")] * 3
        o.velocity_variance = [float("nan")] * 3
        o.reset_counter = self.reset_counter
        self.pub.publish(o)

    def _yaw_meas(self, frame):
        """B (2026-11-14): 하향 영상 직선들의 지면 방향 -> 격자 축(0/90°)과의 차 = EKF 헤딩 오차.
        EKF 자세로 투영하므로 EKF yaw가 실제보다 d 크면 선이 +d 돌아 보임 -> 실제 yaw = EKF yaw - d.
        ±45°까지 받음 (선 위치 측정의 25° 허용각과 별개). 분산은 오차 크기에 비례(큰 오차도 게이트 통과)."""
        cam = self.fl.cams["down"]
        segs, _ = detect_segments(frame, cam.s)
        R = quat_to_rot(self.q)
        h_body = self.lp.dist_bottom + 0.10
        devs, ws = [], []
        for x1, y1, x2, y2, dk, n in segs:
            p1 = self.fl._ground(cam, x1, y1, R, h_body)
            p2 = self.fl._ground(cam, x2, y2, R, h_body)
            if p1 is None or p2 is None:
                continue
            dn, de = p2[0] - p1[0], p2[1] - p1[1]
            L = math.hypot(dn, de)
            if L < 0.3:
                continue
            a = math.atan2(de, dn)
            devs.append((a + math.pi / 4) % (math.pi / 2) - math.pi / 4); ws.append(L)
        if len(devs) < 4:
            self._why("yaw_few"); return None
        devs, ws = np.array(devs), np.array(ws)
        o = np.argsort(devs); cw = np.cumsum(ws[o]); d = float(devs[o][np.searchsorted(cw, cw[-1] / 2)])
        inl = np.abs(devs - d) < math.radians(3)
        if ws[inl].sum() < 0.6 * ws.sum():
            self._why("yaw_spread"); return None
        d = float(np.average(devs[inl], weights=ws[inl]))
        w, x, y, z = self.q
        ekf_yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        yaw = (ekf_yaw - d + math.pi) % (2 * math.pi) - math.pi
        sd = max(abs(d) / 2, math.radians(2))
        self._why("yaw_ok")
        self.ydev_pub.publish(Float32(data=float(math.degrees(d))))
        return yaw, sd * sd

    def _fused_meas(self, frame, prior_n, prior_e, meas_n, meas_e, used):
        """윤곽 쌍 검출(fused_line, 하향) -> 남북선 E / 동서선 N 측정. yaw=0으로 넣어 F=dN, R=dE."""
        h_body = self.lp.dist_bottom + 0.10          # 라이다 z -0.10
        edges = self.fl.edges("down", frame, self.q, h_body, 0.0)
        ns, ew = FusedLine.pair(edges, "along"), FusedLine.pair(edges, "cross")
        if not ns and not ew:
            self._why("no_lines")
        for kind, lines in (("E", ns), ("N", ew)):
            for l in lines:
                P = np.array(l["pts"])
                if math.hypot(prior_n + P[:, 0].mean(), prior_e + P[:, 1].mean()) < PAD_LINE_EXCLUDE_R:
                    self._why("pad"); continue
                rel = l["icpt"]
                prior = prior_e if kind == "E" else prior_n
                g = snap(prior + rel, X_LINES if kind == "E" else Y_LINES)
                if g is None:
                    self._why(f"snap{kind}_fail")
                elif abs(g - rel - prior) >= self.max_innov:
                    self._why(f"innov{kind}")
                else:
                    (meas_e if kind == "E" else meas_n).append(g - rel)
                    used.append(("fused", kind, g))

    def publish_debug(self, frame, used, meas_n, meas_e, pn, pe, header):
        if self.dbg.get_subscription_count() == 0:
            return
        vis = frame.copy()
        for u in used:
            if u[0] == "fused":
                continue
            rho, th, ax, g = u
            a, b = math.cos(th), math.sin(th)
            x0, y0 = a * rho, b * rho
            p1 = (int(x0 - 1000 * b), int(y0 + 1000 * a))
            p2 = (int(x0 + 1000 * b), int(y0 - 1000 * a))
            cv2.line(vis, p1, p2, (0, 255, 0) if ax == "E" else (255, 128, 0), 2)
        cv2.drawMarker(vis, (int(self.cx), int(self.cy)), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
        t1 = f"line: E={statistics.median(meas_e):6.2f}" if meas_e else "line: E=  --  "
        t1 += f" N={statistics.median(meas_n):6.2f}" if meas_n else " N=  --  "
        t2 = f"EKF : E={pe:6.2f} N={pn:6.2f}   (world x=E, y=N)"
        cv2.putText(vis, t1, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(vis, t2, (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        out = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        out.header = header
        self.dbg.publish(out)

    def _why(self, k):
        self.why[k] = self.why.get(k, 0) + 1

    def report(self):
        if self.why and self.lp is not None:
            self.get_logger().info(f"[why] E={self.lp.y:.1f} N={self.lp.x:.1f} h={self.lp.dist_bottom:.2f} {self.why}")
        self.why = {}
        if self._reloc_chk:
            c = self._reloc_chk
            ids = sorted({x[0] for x in c})
            self.get_logger().info(
                f"[RELOC-chk] ArUco 절대위치-EKF: n={len(c)} ids={ids} dN 중앙 {np.median([x[1] for x in c]):+.3f} "
                f"dE 중앙 {np.median([x[2] for x in c]):+.3f} 최대 {max(math.hypot(x[1], x[2]) for x in c):.2f} m")
            self._reloc_chk = []
        s = self.stats
        if s["n"] == 0:
            return
        def st(v):
            return f"{np.mean(v):+.3f}±{np.std(v):.3f} (n={len(v)})" if v else "-"
        self.get_logger().info(f"[line-EKF] frames {s['n']}  dE {st(s['de'])}  dN {st(s['dn'])}")
        self.stats = {"n": 0, "dn": [], "de": []}


def main():
    rclpy.init()
    rclpy.spin(LineOdometry())


if __name__ == "__main__":
    main()
