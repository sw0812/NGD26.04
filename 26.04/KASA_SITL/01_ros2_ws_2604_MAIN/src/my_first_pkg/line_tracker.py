#!/usr/bin/env python3
"""하향 카메라 이진화 기반 구획선(라인) 검출 로직.

search_phase_node.py의 비행 제어 로직과 분리해서, C270 같은 일반 웹캠으로도
(ROS2/PX4 없이) 곧바로 검출 품질을 확인할 수 있게 뺐다. 실제 사용처는
search_phase_node.py이고, test_line_tracker_camera.py는 이 모듈을 그대로
가져다 눈으로 확인하는 용도.
"""
import math

import cv2
import numpy as np


class LineTrackResult:
    __slots__ = ("line_found", "e_y", "e_psi", "intersection")

    def __init__(self, line_found, e_y, e_psi, intersection):
        self.line_found = line_found
        self.e_y = e_y            # 횡방향 편차(px, 화면 중앙 기준, +면 라인이 오른쪽)
        self.e_psi = e_psi        # 각도 편차(rad, ROI 상단/하단 중심 offset으로 추정)
        self.intersection = intersection  # 교차점(십자) 검출 여부


class LineTracker:
    """하향 카메라 이진화 기반 라인트레이싱. 격자선이 바닥과 대비되는 색이라는
    규정 전제를 그대로 이용 — Otsu 이진화로 색상 확정 없이도 라인 대 바닥을
    분리할 수 있게 함."""

    def __init__(self, roi_top_ratio=0.55, roi_bottom_ratio=0.95, cross_row_ratio=0.5):
        self.roi_top_ratio = roi_top_ratio
        self.roi_bottom_ratio = roi_bottom_ratio
        self.cross_row_ratio = cross_row_ratio  # 교차점 판정용 가로 스캔 위치(ROI 내부)
        # 직전 프레임에서 찾은 위치 - 연속성 판단용(2026-09-07 실측: 발처럼
        # 라인과 비슷한 굵기의 물체가 화면에 새로 나타나면 그쪽으로 튀는
        # 문제가 있어서, "이전 위치와 가까운 쪽"을 우선하도록 함).
        self._last_bottom_x = None
        self._last_top_x = None
        # 두께 가중 평활화(EMA) 상태 - 얇고(신뢰도 낮은) 검출일수록 출력을
        # 덜 바꾸고, 두꺼울수록(신뢰도 높을수록) 빨리 반영해서 흔들림/노이즈
        # 로 인한 e_y·e_psi 떨림을 줄인다.
        self._smoothed_e_y = None
        self._smoothed_e_psi = None
        self._fit = None          # 직전 프레임 피팅 직선 (x = a*y + b, ROI 좌표)
        self._hold_count = 0
        self._recent_e_y = []

    @staticmethod
    def _binarize(gray):
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # 라인이 바닥보다 밝을 수도 어두울 수도 있어서, 흰 픽셀이 더 적은 쪽을
        # "라인"으로 취급 (라인 폭 10cm << 바닥 면적이라는 전제).
        if cv2.countNonZero(mask) > mask.size // 2:
            mask = cv2.bitwise_not(mask)
        return mask

    def _row_centroid(self, mask_row, prefer_x=None):
        """반환값: (중심 x, 폭 px) 또는 못 찾으면 None."""
        xs = np.nonzero(mask_row)[0]
        if xs.size == 0:
            return None
        # 전체 픽셀 평균 대신, 가장 넓은 "연속된" 구간의 중심을 쓴다.
        # 평균은 바닥 텍스처/그라우트선 같은 잡음이 섞이면 실제 라인이
        # 아니라 잡음 쪽으로 중심이 쏠리는 문제가 실측됨(2026-09-07,
        # 화강암 타일 바닥에서 검은 테이프 대신 타일 이음새에 점이
        # 찍힘). 라인은 화면에서 가장 굵은 연속 어두운/밝은 구간이라는
        # 전제가 더 안전하다.
        #
        # 다만 "무조건 가장 넓은 구간"만 고르면, 렌즈 비네팅/그림자처럼
        # 라인보다 훨씬 넓은 어두운 덩어리가 있을 때 그쪽을 잘못 고르는
        # 문제가 또 실측됨(2026-09-07, 같은 세션) -> 라인이라고 믿을만한
        # 폭 상한을 두고, 그 상한을 넘는 구간만 있으면 차라리 "못 찾음"
        # 으로 처리한다(엉뚱한 위치를 자신있게 보고하는 것보다 안전).
        gaps = np.where(np.diff(xs) > 1)[0]
        starts = np.concatenate(([0], gaps + 1))
        ends = np.concatenate((gaps, [len(xs) - 1]))
        widths = xs[ends] - xs[starts] + 1

        max_reasonable_width = max(20, int(0.15 * len(mask_row)))
        valid = widths <= max_reasonable_width
        if not np.any(valid):
            return None

        valid_idx = np.where(valid)[0]
        if prefer_x is not None:
            # 발처럼 라인과 비슷한 굵기의 물체가 화면에 새로 나타나면
            # (2026-09-07 실측) 폭 기준만으로는 못 걸러서, 후보가 여럿이면
            # 직전 프레임 위치와 가장 가까운 쪽을 우선한다(연속성 가정 -
            # 라인은 프레임 사이 위치가 크게 안 튐).
            centers = (xs[starts[valid_idx]] + xs[ends[valid_idx]]) / 2.0
            best = valid_idx[int(np.argmin(np.abs(centers - prefer_x)))]
        else:
            best = valid_idx[np.argmax(widths[valid_idx])]
        run = xs[starts[best]:ends[best] + 1]
        width = int(run[-1] - run[0] + 1)
        return float(run.mean()), width

    # ---- 실기체용 안정화 파라미터 (2026-09-29: 2개 행만 쓰던 방식이 프레임마다
    # 튀는 문제 -> ROI 전체 행을 모아 직선 피팅 + 점프 게이팅으로 교체) ----
    N_ROWS = 14              # ROI를 이만큼의 행으로 샘플링
    MIN_ROWS = 4             # 이보다 적은 행에서만 라인이 잡히면 검출 실패로 봄
    FIT_INLIER_PX_RATIO = 0.03   # 피팅 잔차가 폭의 이 비율(최소 6px) 넘으면 이상치
    JUMP_GATE_RATIO = 0.20   # 프레임 간 e_y 변화가 폭의 이 비율을 넘으면 점프로 판정
    PSI_JUMP_GATE = math.radians(35)  # e_psi 점프 판정
    MAX_HOLD_FRAMES = 4      # 점프/일시 소실 시 직전 값을 유지할 최대 프레임 수

    def _collect_row_points(self, mask, w):
        """ROI 마스크의 N_ROWS개 행에서 (x, y, width)를 모은다. 직전 피팅선이
        있으면 그 행에서 예측한 x에 가까운 후보를 우선(연속성)."""
        h = mask.shape[0]
        ys = np.linspace(min(5, h - 1), max(h - 6, 0), self.N_ROWS).astype(int)
        pts = []
        for y in ys:
            prefer = None
            if self._fit is not None:
                a, b = self._fit
                prefer = a * y + b
            r = self._row_centroid(mask[y, :], prefer_x=prefer)
            if r is not None:
                pts.append((r[0], float(y), r[1]))
        return pts

    @staticmethod
    def _fit_line(pts):
        """x = a*y + b 를 Huber 손실로 피팅(이상치에 강함). 반환 (a, b)."""
        arr = np.array([[p[0], p[1]] for p in pts], dtype=np.float32)
        vx, vy, x0, y0 = cv2.fitLine(arr, cv2.DIST_HUBER, 0, 0.01, 0.01).flatten()
        if abs(vy) < 1e-6:
            return None
        a = vx / vy
        return float(a), float(x0 - a * y0)

    def _reject_and_refit(self, pts, w):
        fit = self._fit_line(pts)
        if fit is None:
            return None, 0
        a, b = fit
        tol = max(6.0, self.FIT_INLIER_PX_RATIO * w)
        inl = [p for p in pts if abs(p[0] - (a * p[1] + b)) <= tol]
        if len(inl) < self.MIN_ROWS:
            return None, len(inl)
        if len(inl) != len(pts):
            fit = self._fit_line(inl) or fit
        return fit, len(inl)

    def _hold_or_lose(self):
        """이번 프레임 값을 못 믿을 때: 직전 값을 잠깐 유지, 오래되면 검출 실패."""
        self._hold_count += 1
        if self._smoothed_e_y is not None and self._hold_count <= self.MAX_HOLD_FRAMES:
            return LineTrackResult(True, self._smoothed_e_y, self._smoothed_e_psi, False)
        self._reset_state()
        return LineTrackResult(False, 0.0, 0.0, False)

    def _reset_state(self):
        self._last_bottom_x = self._last_top_x = None
        self._smoothed_e_y = self._smoothed_e_psi = None
        self._fit = None
        self._hold_count = 0
        self._recent_e_y = []

    def process(self, bgr_image) -> LineTrackResult:
        h, w = bgr_image.shape[:2]
        y0, y1 = int(h * self.roi_top_ratio), int(h * self.roi_bottom_ratio)
        roi = bgr_image[y0:y1, :]
        gray = cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        mask = self._binarize(gray)
        # 가는 노이즈 점 제거(타일 이음새 등)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        pts = self._collect_row_points(mask, w)
        fit, n_inl = (None, 0)
        if len(pts) >= self.MIN_ROWS:
            fit, n_inl = self._reject_and_refit(pts, w)
        if fit is None:
            return self._hold_or_lose()

        a, b = fit
        hh = mask.shape[0]
        y_bot, y_top = max(hh - 5, 0), min(5, hh - 1)
        bottom_c, top_c = a * y_bot + b, a * y_top + b
        dy_px = max(1, y_bot - y_top)
        raw_e_y = bottom_c - (w / 2.0)
        raw_e_psi = math.atan2(top_c - bottom_c, dy_px)

        # 점프 게이팅: 직전 값에서 갑자기 크게 벗어난 프레임은 잠시 무시(발/그림자/
        # 옆 격자선으로 튀는 경우). MAX_HOLD_FRAMES 넘게 계속되면 진짜 이동으로 수용.
        if self._smoothed_e_y is not None and self._hold_count < self.MAX_HOLD_FRAMES:
            if (abs(raw_e_y - self._smoothed_e_y) > self.JUMP_GATE_RATIO * w
                    or abs(raw_e_psi - self._smoothed_e_psi) > self.PSI_JUMP_GATE):
                self._hold_count += 1
                return LineTrackResult(True, self._smoothed_e_y, self._smoothed_e_psi,
                                       self._detect_intersection(mask))
        self._hold_count = 0

        self._fit = fit
        self._last_bottom_x, self._last_top_x = bottom_c, top_c

        # 최근 3프레임 중앙값으로 순간 스파이크 제거 후 EMA
        self._recent_e_y.append(raw_e_y)
        self._recent_e_y = self._recent_e_y[-3:]
        med_e_y = float(np.median(self._recent_e_y))

        # 피팅에 쓰인 행 수 = 신뢰도. 행이 많이 잡힐수록 새 값을 빨리 반영.
        confidence = min(1.0, n_inl / self.N_ROWS)
        alpha = 0.25 + 0.45 * confidence  # 0.25~0.70 (기존 0.2~0.8보다 완만)
        if self._smoothed_e_y is None:
            self._smoothed_e_y, self._smoothed_e_psi = med_e_y, raw_e_psi
        else:
            self._smoothed_e_y = alpha * med_e_y + (1 - alpha) * self._smoothed_e_y
            self._smoothed_e_psi = alpha * raw_e_psi + (1 - alpha) * self._smoothed_e_psi

        return LineTrackResult(True, self._smoothed_e_y, self._smoothed_e_psi,
                               self._detect_intersection(mask))

    def _detect_intersection(self, mask):
        """ROI 중간 높이에서 가로로 스캔했을 때 라인 폭이 정상 구획선 폭보다
        훨씬 넓게(또는 라인 구간이 2개 이상) 잡히면 교차점(십자)으로 판정.
        정상 라인트레이싱 중에는 세로선 하나만 폭 좁게 잡혀야 함."""
        row_idx = int(mask.shape[0] * self.cross_row_ratio)
        row = mask[row_idx, :]
        xs = np.nonzero(row)[0]
        if xs.size == 0:
            return False
        span = xs.max() - xs.min()
        # ROI 폭의 60% 이상을 라인이 덮으면 가로줄과 겹친 교차점으로 판단.
        return span > 0.6 * mask.shape[1]
