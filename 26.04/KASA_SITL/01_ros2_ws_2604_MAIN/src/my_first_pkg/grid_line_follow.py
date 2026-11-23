#!/usr/bin/env python3
"""격자 구획선 기반 라인트레이싱 비전 (GNC용).

line_tracker.py는 "화면에 선이 하나" 가정이라, 3m 격자가 2~3줄씩 보이는 하향 광각
카메라(gz SITL HFOV 115°)에선 엉뚱한 선/가로선/기체 다리 그림자/ArUco를 잡음
(2026-11-10 비행 캡처: e_psi 1.3rad 등). 여기선:

  1) black-hat으로 "주변보다 어두운 가는 선"만 추출 (흰/베이지 바닥 공통)
  2) 큰 덩어리(ArUco·그림자) 제거, 화면 중앙 기체 다리 영역 마스킹
  3) HoughLines -> 진행방향 선(화면 세로 ±35°)과 교차선(가로 ±35°)로 분류
  4) 화면 중심(=기체 바로 아래)에 가장 가까운 세로선 -> 횡오차 e_y, 각도오차 e_psi
     가장 가까운 가로선 -> 교차점까지 전방거리 cross_ahead

좌표 규약 (카메라 축 확인 완료: 화면 위 = 기체 전방, 화면 오른쪽 = 기체 오른쪽):
  e_y_px   > 0 : 선이 기체 오른쪽          -> 오른쪽으로 가야 함
  e_psi    > 0 : 선 윗부분이 오른쪽으로 기움 -> 시계방향(NED yaw +)으로 돌려야 함
  cross_px > 0 : 교차선이 기체 앞쪽
"""
import math

import cv2
import numpy as np


class GridLineResult:
    __slots__ = ("along_found", "e_y_px", "e_psi", "cross_found", "cross_px", "n_lines")

    def __init__(self):
        self.along_found = False
        self.e_y_px = 0.0
        self.e_psi = 0.0
        self.cross_found = False
        self.cross_px = 0.0
        self.n_lines = 0


class GridLineFollower:
    ANGLE_TOL = math.radians(35)
    MERGE_PX = 25           # 같은 선으로 보는 거리
    MERGE_ANG = math.radians(8)

    def __init__(self, cx=320.0, cy=180.0, body_mask_radius=60, body_mask_center=None):
        self.cx, self.cy = cx, cy
        self.body_mask_radius = body_mask_radius
        # 기체 다리/프로펠러가 비치는 위치 (gz 캡처 기준 화면 중앙 약간 아래)
        self.body_mask_center = body_mask_center or (int(cx), int(cy + 85))
        self.last_mask = None

    def _line_mask(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
        bh = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, k)
        # 2026-11-11: ArUco 마커(진한 흑백)가 있으면 Otsu가 마커 기준(~52)으로 잡혀 연한 격자선(~20-27)이
        # 전부 사라짐(배치 C에서 마커 칸마다 선 0개 -> 라인 위치 끊김). 진한 값을 잘라서 임계값 계산.
        t, _ = cv2.threshold(np.minimum(bh, 40).astype(np.uint8), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _, m = cv2.threshold(bh, max(t, 8), 255, cv2.THRESH_BINARY)
        if cv2.countNonZero(m) > 0.25 * m.size:   # 선이 없는 화면에서 Otsu가 잡음을 다 잡는 경우
            _, m = cv2.threshold(bh, 25, 255, cv2.THRESH_BINARY)
        # 큰 덩어리(ArUco 마커, 그림자) 제거: 가늘고 긴 것만 남김
        n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
        keep = np.zeros(n, np.uint8)
        for i in range(1, n):
            w, h, a = st[i, 2], st[i, 3], st[i, 4]
            if a < 40:
                continue
            fill = a / float(w * h)
            if min(w, h) > 18 and fill > 0.25:   # 꽉 찬 사각형 덩어리 = 마커/그림자
                continue
            keep[i] = 255
        m = keep[lab]
        cv2.circle(m, self.body_mask_center, self.body_mask_radius, 0, -1)
        return m

    def _merge(self, lines):
        out = []
        for rho, th in lines:
            if any(abs(rho - r) < self.MERGE_PX and abs(th - t) < self.MERGE_ANG for r, t in out):
                continue
            out.append((rho, th))
        return out

    def lines(self, bgr):
        """검출된 격자선 목록 [(rho, theta)], theta는 (-pi/2, pi/2]."""
        m = self._line_mask(bgr)
        self.last_mask = m
        hl = cv2.HoughLines(m, 1, math.pi / 180, 90)
        if hl is None:
            return []
        lines = []
        for rho, th in hl[:, 0, :]:
            # theta를 (-pi/2, pi/2]로 정규화 (rho 부호 같이 뒤집음)
            if th > math.pi / 2:
                th -= math.pi
                rho = -rho
            lines.append((float(rho), float(th)))
        return self._merge(lines)

    def process(self, bgr):
        r = GridLineResult()
        lines = self.lines(bgr)
        if not lines:
            return r
        r.n_lines = len(lines)
        along, cross = [], []
        for rho, th in lines:
            if abs(th) < self.ANGLE_TOL:                       # 화면 세로선
                x = (rho - self.cy * math.sin(th)) / math.cos(th)
                along.append((x - self.cx, th))
            elif abs(abs(th) - math.pi / 2) < self.ANGLE_TOL:  # 화면 가로선
                y = (rho - self.cx * math.cos(th)) / math.sin(th)
                cross.append(self.cy - y)
        if along:
            e_y, th = min(along, key=lambda a: abs(a[0]))
            r.along_found, r.e_y_px, r.e_psi = True, e_y, th
        if cross:
            r.cross_found, r.cross_px = True, min(cross, key=abs)
        return r


def draw(bgr, res, follower):
    vis = bgr.copy()
    h, w = vis.shape[:2]
    cx, cy = int(follower.cx), int(follower.cy)
    cv2.drawMarker(vis, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
    cv2.circle(vis, follower.body_mask_center, follower.body_mask_radius, (128, 128, 128), 1)
    if res.along_found:
        x0 = cx + res.e_y_px
        dx = math.tan(res.e_psi) * h
        cv2.line(vis, (int(x0 + math.tan(res.e_psi) * cy), 0),
                 (int(x0 - math.tan(res.e_psi) * (h - cy)), h), (0, 255, 0), 2)
    if res.cross_found:
        yy = int(cy - res.cross_px)
        cv2.line(vis, (0, yy), (w, yy), (0, 0, 255), 2)
    txt = (f"along {'Y' if res.along_found else 'N'} e_y={res.e_y_px:+.0f}px "
           f"e_psi={math.degrees(res.e_psi):+.1f}deg | cross {'Y' if res.cross_found else 'N'} "
           f"{res.cross_px:+.0f}px")
    cv2.putText(vis, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    if follower.last_mask is not None:
        sm = cv2.resize(cv2.cvtColor(follower.last_mask, cv2.COLOR_GRAY2BGR), (w // 4, h // 4))
        vis[h - sm.shape[0]:h, w - sm.shape[1]:w] = sm
    return vis


if __name__ == "__main__":
    import sys
    f = GridLineFollower()
    for p in sys.argv[1:]:
        img = cv2.imread(p)
        res = f.process(img)
        print(p.split('/')[-1], 'lines', res.n_lines, 'along', res.along_found, round(res.e_y_px),
              round(math.degrees(res.e_psi), 1), 'cross', res.cross_found, round(res.cross_px))
        cv2.imwrite(p.replace('.png', '_grid.png'), draw(img, res, f))
