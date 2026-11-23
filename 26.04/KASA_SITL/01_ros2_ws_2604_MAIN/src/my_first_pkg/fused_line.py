#!/usr/bin/env python3
"""전방+하향 카메라 격자선 융합 측정 (2026-11-11, 측정 전용 — 비행 제어 안 씀).

픽셀 굵기 기준(blackhat 커널) 대신:
  1) Canny 윤곽선 -> HoughLinesP 선분
  2) 선분마다 어두운 쪽(극성) 판정
  3) 카메라 광선 -> 기체 자세 -> 지면 교차로 기수 기준 (F 전방, R 오른쪽) [m] 투영
  4) 지면에서 "밝->어둠" + "어둠->밝" 윤곽 쌍(폭 W_MIN~W_MAX m)만 페인트 선으로 인정
     (블록 무늬 경계 같은 외톨이 윤곽은 버림). 미터 기준이라 렌즈/해상도 바뀌어도 그대로.
  5) along 선: 하향(발밑)+전방(2~4 m 앞) 점을 한 직선 R = lat + tan(psi)*F 로 최소제곱
     -> 기준선이 길어 psi(기수오차) 정확. cross 선: F 절편.

부호: lat>0 선이 오른쪽, psi>0 선 앞쪽이 오른쪽으로 기움(시계방향 yaw+ 필요), fwd>0 앞쪽.
"""
import math

import cv2
import numpy as np

AXIS_TOL = math.radians(25)
W_MIN, W_MAX = 0.03, 0.25      # m, 페인트 선 폭 (실측 0.10 m)
CLUSTER = 0.30                 # m, 같은 선으로 묶는 거리


def quat_to_rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class Cam:
    """pitch: 광축이 기체 전방에서 아래로 기운 각 [rad] (하향=pi/2, 전방=0.6109).
    offset: 기체 원점 기준 카메라 위치, body FRD [m]."""

    def __init__(self, fx, cx, cy, pitch, offset, scale=0.5):
        self.s = scale
        self.fx, self.cx, self.cy = fx * scale, cx * scale, cy * scale
        a = pitch
        self.Z = np.array([math.cos(a), 0.0, math.sin(a)])    # 광축
        self.Y = np.array([-math.sin(a), 0.0, math.cos(a)])   # 화면 아래
        self.X = np.array([0.0, 1.0, 0.0])                    # 화면 오른쪽
        self.off = np.array(offset, float)

    def ray(self, u, v):
        return (u - self.cx) / self.fx * self.X + (v - self.cy) / self.fx * self.Y + self.Z


def detect_segments(bgr, scale=0.5):
    """반환: [(x1,y1,x2,y2, dark_sign)], 좌표는 축소 영상 기준.
    dark_sign: 선분 법선(+n = 방향 d를 90도 돌린 쪽)에서 +n 쪽이 어두우면 +1."""
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    if scale != 1.0:
        g = cv2.resize(g, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    g = cv2.GaussianBlur(g, (5, 5), 0)
    e = cv2.Canny(g, 15, 40)
    H, W = g.shape
    segs = cv2.HoughLinesP(e, 1, math.pi / 360, 40, minLineLength=int(0.12 * min(H, W)), maxLineGap=8)
    out = []
    if segs is None:
        return out, g
    for x1, y1, x2, y2 in segs[:, 0, :]:
        d = np.array([x2 - x1, y2 - y1], float)
        L = np.linalg.norm(d)
        if L < 1:
            continue
        d /= L
        n = np.array([-d[1], d[0]])
        vals = []
        for t in (0.25, 0.5, 0.75):
            p = np.array([x1, y1]) + t * L * d
            a = p + 4 * n
            b = p - 4 * n
            if 0 <= a[0] < W and 0 <= a[1] < H and 0 <= b[0] < W and 0 <= b[1] < H:
                vals.append(float(g[int(a[1]), int(a[0])]) - float(g[int(b[1]), int(b[0])]))
        if not vals:
            continue
        m = float(np.median(vals))
        if abs(m) < 6:
            continue
        out.append((x1, y1, x2, y2, -1 if m > 0 else 1, n))
    return out, g


class FusedLine:
    def __init__(self, down: Cam, front: Cam):
        self.cams = {"down": down, "front": front}

    def _ground(self, cam, u, v, R, h_body):
        """h_body: 기체 원점의 지면 높이 [m]. 반환 NED 수평 (dN, dE) 기체 원점 기준."""
        r = R @ cam.ray(u, v)
        if r[2] < 0.15:
            return None
        cpos = R @ cam.off
        hc = h_body - cpos[2]           # cpos[2] NED(+아래): 카메라가 아래 있으면 지면까지 더 가까움
        if hc <= 0.1:
            return None
        t = hc / r[2]
        return cpos[0] + t * r[0], cpos[1] + t * r[1]

    def edges(self, name, bgr, q, h_body, yaw):
        """한 카메라의 지면 윤곽선 목록: dict(kind, icpt, ang, dark, pts)."""
        cam = self.cams[name]
        segs, _ = detect_segments(bgr, cam.s)
        R = quat_to_rot(q)
        c, s = math.cos(yaw), math.sin(yaw)
        out = []
        for x1, y1, x2, y2, dk, n in segs:
            p1 = self._ground(cam, x1, y1, R, h_body)
            p2 = self._ground(cam, x2, y2, R, h_body)
            mid = ((x1 + x2) / 2, (y1 + y2) / 2)
            pd = self._ground(cam, mid[0] + dk * 4 * n[0], mid[1] + dk * 4 * n[1], R, h_body)
            pm = self._ground(cam, *mid, R, h_body)
            if None in (p1, p2, pd, pm):
                continue
            fr = lambda p: (p[0] * c + p[1] * s, -p[0] * s + p[1] * c)
            (f1, r1), (f2, r2), (fd, rd), (fm, rm) = fr(p1), fr(p2), fr(pd), fr(pm)
            df, dr = f2 - f1, r2 - r1
            a = math.atan2(dr, df)
            a = (a + math.pi / 2) % math.pi - math.pi / 2
            if abs(a) < AXIS_TOL and abs(df) > 1e-6:
                icpt = r1 - f1 * dr / df                     # F=0에서 R
                dark = 1 if rd > rm else -1                   # 어두운 쪽이 +R?
                out.append(dict(kind="along", icpt=icpt, ang=a, dark=dark, pts=[(f1, r1), (f2, r2)], cam=name))
            elif abs(abs(a) - math.pi / 2) < AXIS_TOL and abs(dr) > 1e-6:
                icpt = f1 - r1 * df / dr                     # R=0에서 F
                dark = 1 if fd > fm else -1
                out.append(dict(kind="cross", icpt=icpt, ang=a, dark=dark, pts=[(f1, r1), (f2, r2)], cam=name))
        return out

    @staticmethod
    def pair(edges, kind):
        """밝->어둠(dark=+1, 낮은 쪽 경계) + 어둠->밝(dark=-1, 높은 쪽 경계) 쌍 -> 선 중심."""
        lo = [e for e in edges if e["kind"] == kind and e["dark"] == 1]
        hi = [e for e in edges if e["kind"] == kind and e["dark"] == -1]
        lines = []
        for a in lo:
            best = None
            for b in hi:
                w = b["icpt"] - a["icpt"]
                if W_MIN <= w <= W_MAX and abs(a["ang"] - b["ang"]) < math.radians(10):
                    if best is None or w < best[0]:
                        best = (w, b)
            if best:
                b = best[1]
                lines.append(dict(icpt=(a["icpt"] + b["icpt"]) / 2, width=best[0],
                                  pts=a["pts"] + b["pts"], cams={a["cam"], b["cam"]}))
        return lines

    @staticmethod
    def fit_along(lines, cams=None):
        """lat 0 근처(가장 가까운) 선 묶음 -> (lat, psi, n점, 쓴 카메라)."""
        ls = [l for l in lines if cams is None or l["cams"] & cams]
        if not ls:
            return None
        ls.sort(key=lambda l: abs(l["icpt"]))
        ref = ls[0]["icpt"]
        grp = [l for l in ls if abs(l["icpt"] - ref) < CLUSTER]
        P = np.array([p for l in grp for p in l["pts"]])
        if len(P) < 2:
            return None
        A = np.c_[np.ones(len(P)), P[:, 0]]
        sol, *_ = np.linalg.lstsq(A, P[:, 1], rcond=None)
        if np.ptp(P[:, 0]) < 0.15:     # F 범위가 너무 짧으면 기울기 못 믿음 -> 절편만
            return float(np.median([l["icpt"] for l in grp])), None, len(P), set().union(*[l["cams"] for l in grp])
        return float(sol[0]), float(math.atan(sol[1])), len(P), set().union(*[l["cams"] for l in grp])

    @staticmethod
    def crosses(lines):
        fs = sorted(l["icpt"] for l in lines)
        g = []
        for f in fs:
            if g and f - g[-1][-1] < CLUSTER:
                g[-1].append(f)
            else:
                g.append([f])
        return [sum(x) / len(x) for x in g]
