#!/usr/bin/env python3
"""확률적 허프 변환(cv2.HoughLinesP)으로 KASA 필드 구획선을 검출하고,
waypoints.json에 이미 확정돼 있는 격자 좌표(월드 m)를 정답(ground truth)으로 써서
검출 정확도를 검증한다.

배경: search_phase_node.py의 LineTracker._binarize()는 Otsu 임계값 기반
row-centroid 방식만 쓰고 있고, 구획선을 실제로 잘 잡는지는 미검증 상태였음
(saslobssw "08 탐색 페이즈 노드" 노트의 TODO #3). 실카메라 프레임이 아직 없으니
Gazebo에 실제로 쓰인 field_texture.png(=실측 축척 그대로 만든 바닥 텍스처)를
대신 써서, "정확한 좌표를 안다"는 이점을 이용해 파라미터를 검증 가능한
방식으로 튜닝한다.

좌표 변환: waypoints.json은 픽셀 스케일(texture_scale_px_per_m)과
safe_area의 world 범위를 담고 있어서, 아래 두 식만으로 world(m) <-> 텍스처
픽셀을 왕복 변환할 수 있다 (generate_field.py의 g2i()와 동일한 매핑을
world_x/y_range로부터 역산한 것):

    px = (world_x - safe_area.x_range[0]) * scale
    py = (safe_area.y_range[1] - world_y) * scale
"""
import json
import math
import os

import cv2
import numpy as np

BASE = os.path.dirname(os.path.abspath(__file__))
WAYPOINTS_JSON = os.path.expanduser(
    "~/Downloads/kasa_gazebo_world/models/kasa_ground_field/waypoints.json")
FIELD_TEXTURE = os.path.expanduser(
    "~/Downloads/kasa_gazebo_world/models/kasa_ground_field/materials/textures/field_texture.png")
OUT_DIR = os.path.join(BASE, "results")

ANGLE_TOL_DEG = 5.0      # 수평/수직 판정 허용각
PREVIEW_WIDTH = 1600     # 오버레이 미리보기 가로 픽셀


def load_field():
    with open(WAYPOINTS_JSON) as f:
        return json.load(f)


def world_to_px(x, y, field):
    scale = field["texture_scale_px_per_m"]
    x0 = field["safe_area"]["x_range"][0]
    y1 = field["safe_area"]["y_range"][1]
    return (x - x0) * scale, (y1 - y) * scale


def ground_truth_lines(field):
    """mission_area 격자선을 (x1,y1,x2,y2,orientation,coord_m) 픽셀 리스트로."""
    mis = field["mission_area"]
    spacing = field["grid"]["spacing"]
    x0, x1 = mis["x_range"]
    y0, y1 = mis["y_range"]
    n_cols = int(round((x1 - x0) / spacing))
    n_rows = int(round((y1 - y0) / spacing))

    lines = []
    for i in range(n_cols + 1):
        x = x0 + i * spacing
        p0 = world_to_px(x, y0, field)
        p1 = world_to_px(x, y1, field)
        lines.append((*p0, *p1, "V", x))
    for j in range(n_rows + 1):
        y = y0 + j * spacing
        p0 = world_to_px(x0, y, field)
        p1 = world_to_px(x1, y, field)
        lines.append((*p0, *p1, "H", y))
    return lines


def detect_lines(gray, canny_lo=50, canny_hi=150, hough_threshold=120,
                  min_line_length=150, max_line_gap=30):
    edges = cv2.Canny(gray, canny_lo, canny_hi)
    segs = cv2.HoughLinesP(
        edges, rho=1, theta=np.pi / 180, threshold=hough_threshold,
        minLineLength=min_line_length, maxLineGap=max_line_gap)
    return [] if segs is None else segs.reshape(-1, 4)


def classify(seg):
    x1, y1, x2, y2 = seg
    ang = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180
    if ang < ANGLE_TOL_DEG or ang > 180 - ANGLE_TOL_DEG:
        return "H", (y1 + y2) / 2.0
    if abs(ang - 90) < ANGLE_TOL_DEG:
        return "V", (x1 + x2) / 2.0
    return None, None


def match_against_ground_truth(detections, gt_lines, field):
    """각 정답 격자선마다, 방향/좌표가 근접한 검출 세그먼트가 있는지 확인."""
    line_w_px = field["grid"]["line_width"] * field["texture_scale_px_per_m"]
    tol_px = line_w_px  # 두꺼운 선의 양쪽 엣지가 다 잡히므로 선 폭만큼 여유

    by_orient = {"H": [], "V": []}
    for seg in detections:
        orient, coord = classify(seg)
        if orient:
            by_orient[orient].append(coord)

    results = []
    for (px1, py1, px2, py2, orient, coord_m) in gt_lines:
        gt_px = (py1 + py2) / 2.0 if orient == "H" else (px1 + px2) / 2.0
        candidates = by_orient[orient]
        if candidates:
            nearest = min(candidates, key=lambda c: abs(c - gt_px))
            err_px = abs(nearest - gt_px)
            hit = err_px <= tol_px
        else:
            err_px, hit = float("inf"), False
        results.append({
            "orient": orient, "coord_m": coord_m,
            "err_px": err_px, "err_cm": err_px / field["texture_scale_px_per_m"] * 100,
            "hit": hit,
        })
    return results


def draw_overlay(bgr, gt_lines, detections, field):
    overlay = bgr.copy()
    for (x1, y1, x2, y2, _orient, _coord) in gt_lines:
        cv2.line(overlay, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 6)  # 정답=빨강
    for (x1, y1, x2, y2) in detections:
        cv2.line(overlay, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 3)  # 검출=초록
    scale = PREVIEW_WIDTH / overlay.shape[1]
    preview = cv2.resize(overlay, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return preview


def run(image_path, label):
    field = load_field()
    bgr = cv2.imread(image_path)
    if bgr is None:
        raise FileNotFoundError(image_path)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    gt_lines = ground_truth_lines(field)
    detections = detect_lines(gray)
    results = match_against_ground_truth(detections, gt_lines, field)

    n_hit = sum(r["hit"] for r in results)
    hit_errs = [r["err_cm"] for r in results if r["hit"]]
    print(f"\n=== {label} ({os.path.basename(image_path)}, {bgr.shape[1]}x{bgr.shape[0]}) ===")
    print(f"검출된 선분 수(HoughLinesP): {len(detections)}")
    print(f"정답 격자선 {len(gt_lines)}개 중 검출로 매칭됨: {n_hit}개 "
          f"({100 * n_hit / len(gt_lines):.0f}%)")
    if hit_errs:
        print(f"매칭된 선의 위치 오차: 평균 {sum(hit_errs)/len(hit_errs):.2f}cm, "
              f"최대 {max(hit_errs):.2f}cm  (구획선 폭 {field['grid']['line_width']*100:.0f}cm 기준)")
    missed = [r for r in results if not r["hit"]]
    if missed:
        print("놓친 격자선:", [f"{r['orient']}@{r['coord_m']:.0f}m" for r in missed])

    os.makedirs(OUT_DIR, exist_ok=True)
    out_path = os.path.join(OUT_DIR, f"overlay_{label}.png")
    cv2.imwrite(out_path, draw_overlay(bgr, gt_lines, detections, field))
    print(f"오버레이 저장: {out_path} (빨강=정답 좌표, 초록=허프 검출)")
    return n_hit, len(gt_lines)


if __name__ == "__main__":
    run(FIELD_TEXTURE, "field_texture")
