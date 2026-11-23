#!/usr/bin/env python3
"""라인트레이싱 기반 유도(setpoint)용 측정: 격자선을 기체 진행방향 기준 미터 단위로 변환.

grid_line_follow.GridLineFollower로 Hough 선을 찾고, 각 선의 두 픽셀을
카메라 광선 -> 기체 자세(q, body FRD->NED) -> 지면(높이 h) 교차로 투영한 뒤
기수(yaw) 기준 수평좌표 F(전방), R(오른쪽)로 바꿈 (기울기 보정 포함).

  along 선 (진행방향과 ±30°): lat = F=0에서의 R 오프셋 [m] (+면 선이 오른쪽 -> 오른쪽으로 가야)
                              psi = 선 방향각 [rad] (+면 선이 오른쪽으로 기움 -> 시계방향 yaw +)
  cross 선 (진행방향과 수직 ±30°): fwd = R=0에서의 F 오프셋 [m] (+면 앞쪽)

EKF 위치는 전혀 안 씀 — 위치가 틀려도 측정은 영향 없음.
"""
import math
import os

import numpy as np

from grid_line_follow import GridLineFollower

AXIS_TOL = math.radians(30)


def quat_to_rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class LineMeas:
    __slots__ = ("lat", "psi", "crosses", "n_lines")

    def __init__(self):
        self.lat = None      # 가장 가까운 along 선의 횡오프셋 [m]
        self.psi = None
        self.crosses = []    # 모든 cross 선의 전방거리 [m]
        self.n_lines = 0


class LineGuidance:
    def __init__(self, fx=float(os.environ.get('DOWN_FX', 927.5)), cx=728.0, cy=544.0):  # DOWN_FX: 렌즈 배치용, 기본 3.2mm 927.5 (2026-11-14, 4mm 1159.4)  # 2026-11-11 하향 2.8mm (44.6도 1775.0, 16mm 4637.7, 6mm 1739.1) (이전 205.4696, 320, 180)
        self.fx, self.cx, self.cy = fx, cx, cy
        self.follower = GridLineFollower(cx=cx, cy=cy)

    def _ground(self, u, v, R, h):
        ray = np.array([-(v - self.cy) / self.fx, (u - self.cx) / self.fx, 1.0])  # body FRD
        r = R @ ray
        if r[2] < 0.2:
            return None
        t = h / r[2]
        return t * r[0], t * r[1]   # dN, dE

    def measure(self, bgr, q, h, yaw):
        """q: body->NED quaternion (w,x,y,z), h: 카메라 지면 높이 [m], yaw: NED heading [rad]."""
        m = LineMeas()
        lines = self.follower.lines(bgr)
        m.n_lines = len(lines)
        if not lines:
            return m
        R = quat_to_rot(q)
        c, s = math.cos(yaw), math.sin(yaw)
        best = None
        for rho, th in lines:
            base = np.array([rho * math.cos(th), rho * math.sin(th)])
            d = np.array([-math.sin(th), math.cos(th)])
            foot = base + (np.array([self.cx, self.cy]) - base) @ d * d
            p1 = self._ground(*(foot - 80 * d), R, h)
            p2 = self._ground(*(foot + 80 * d), R, h)
            if p1 is None or p2 is None:
                continue
            # NED -> 기수 기준 (F 전방, R 오른쪽)
            f1, r1 = p1[0] * c + p1[1] * s, -p1[0] * s + p1[1] * c
            f2, r2 = p2[0] * c + p2[1] * s, -p2[0] * s + p2[1] * c
            df, dr = f2 - f1, r2 - r1
            a = math.atan2(dr, df)
            a = (a + math.pi / 2) % math.pi - math.pi / 2       # (-90°, 90°]
            if abs(a) < AXIS_TOL and abs(df) > 1e-6:
                lat = r1 - f1 * dr / df
                if best is None or abs(lat) < abs(best[0]):
                    best = (lat, a)
            elif abs(abs(a) - math.pi / 2) < AXIS_TOL and abs(dr) > 1e-6:
                m.crosses.append(f1 - r1 * df / dr)
        if best:
            m.lat, m.psi = best
        # 같은 교차선이 Hough 직선 여러 개(±0.15m)로 잡힘 -> 0.5m 안은 한 선으로 묶어 평균
        groups = []
        for f in sorted(m.crosses):
            if groups and f - groups[-1][-1] < 0.5:
                groups[-1].append(f)
            else:
                groups.append([f])
        m.crosses = [sum(g) / len(g) for g in groups]
        return m
