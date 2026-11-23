#!/usr/bin/env python3
"""
KASA 중급부문 실내조난자탐색 임무 - Gazebo Classic 시뮬레이션 맵 생성 스크립트

규정집(중급부문) 수치를 1:1 축척으로 재현한다.
모든 텍스처는 동일한 축척 PX_PER_M(px/m) 하나만 사용한다. (배율 불일치 방지)

  Safe area          32 m x 23 m
  Mission area       24 m x 15 m   (safe area 사방 4 m 여유)
  격자(구획선)        3 m x 3 m, 폭 10 cm (TBR)
  경로점 ArUco        50 cm x 50 cm, DICT_5X5_50, ID 1~4, 구획선 교차점 위
  버티포트            지름 3 m, 높이 0.7 m
  항기 안전구역        지름 2 m
  버티포트 중앙 마커    50 cm x 50 cm, ID 0

좌표계 원점(0,0) = safe area(=world) 중심.
(※ 규정에 원점 정의가 없어 임시 지정 — 확정 시 이 값만 바꾸면 전체가 따라감)

출력:
  models/kasa_ground_field/materials/textures/field_texture.png   (32 x 23 m)
  models/vertiport_pad/materials/textures/vertiport_marker.png    (3.2 x 3.2 m)
  models/kasa_ground_field/waypoints.yaml / .json
"""
import cv2
import numpy as np
import json
import os

BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

# ===== 축척 (모든 텍스처 공통) ==========================================
PX_PER_M = 200.0          # px / m   <-- 유일한 축척. 절대 텍스처마다 다르게 쓰지 말 것

def m2px(m):
    """미터 -> 픽셀 (반올림 정수)"""
    return int(round(m * PX_PER_M))

# ===== 규정 제원 =========================================================
SAFE_W,    SAFE_H    = 32.0, 23.0    # safe area (m)
MISSION_W, MISSION_H = 24.0, 15.0    # mission area (m)
GRID        = 3.0     # 격자 간격 (m)
LINE_W      = 0.10    # 구획선 폭 (m) - TBR
MARKER_SIZE = 0.50    # ArUco 마커 한 변 (m)  <-- 경로점/버티포트 공통
VERTIPORT_D = 3.0     # 버티포트 지름 (m)
SAFEZONE_D  = 2.0     # 항기 안전구역 지름 (m)
VERTIPORT_H = 0.7     # 버티포트 높이 (m)

ARUCO_DICT  = cv2.aruco.DICT_5X5_50

# 버티포트 패드 텍스처가 덮는 정사각 범위 (m).
# 지름 3 m 원의 외곽선이 잘리지 않도록 약간의 여백을 준다.
PAD_TEX_M = 3.2

# ===== 색상 (BGR) - 규정집 도식 톤 ======================================
COL_SAFE_BG        = (250, 250, 250)   # safe area 바닥
COL_MISSION_BG     = (170, 214, 235)   # mission area 바닥 (연한 카키/샌드)
COL_GRID_LINE      = (150, 150, 150)   # 구획선
COL_PAD_BG         = (225, 225, 225)   # 버티포트 상판 바탕
COL_VERTIPORT_RING = (0, 0, 0)         # 버티포트 외곽 링 (지름 3 m)
COL_SAFEZONE_RING  = (0, 0, 220)       # 항기 안전구역 링 (지름 2 m)
COL_V_LOGO         = (0, 0, 0)         # V 픽토그램

_audit = []   # (항목, 실측 m, 규정 m)

def audit(name, measured_m, spec_m):
    _audit.append((name, measured_m, spec_m))

# ===== ArUco 그리기 (미터 단위 지정) ====================================
def draw_aruco(img, origin_m, cx, cy, size_m, marker_id):
    """
    img       : 대상 이미지
    origin_m  : (x_min, y_max) 이미지 좌상단이 대응하는 월드 좌표 (m)
    cx, cy    : 마커 중심 월드 좌표 (m)
    size_m    : 마커 한 변 (m)
    """
    side_px = m2px(size_m)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, side_px)
    marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    x_min, y_max = origin_m
    px = m2px(cx - size_m / 2 - x_min)
    py = m2px(y_max - (cy + size_m / 2))
    img[py:py + side_px, px:px + side_px] = marker
    audit(f"ArUco ID{marker_id}", side_px / PX_PER_M, size_m)
    return side_px

# ===== 버티포트 표식 그리기 (미터 단위) =================================
def draw_vertiport(img, origin_m, cx, cy, with_aruco=True):
    """버티포트 표식: 상판 원 + 외곽 링(3m) + 안전구역 링(2m) + V 픽토그램 + 중앙 ArUco(ID0)"""
    x_min, y_max = origin_m
    ox = m2px(cx - x_min)
    oy = m2px(y_max - cy)

    r_outer = m2px(VERTIPORT_D / 2)     # 1.5 m -> 300 px
    r_inner = m2px(SAFEZONE_D / 2)      # 1.0 m -> 200 px
    ring_w  = max(2, m2px(0.06))        # 링 선 굵기 6 cm
    zone_w  = max(2, m2px(0.04))        # 안전구역 링 굵기 4 cm

    # 상판 바탕
    cv2.circle(img, (ox, oy), r_outer, COL_PAD_BG, -1)
    # 외곽 링 (지름 3 m)
    cv2.circle(img, (ox, oy), r_outer, COL_VERTIPORT_RING, thickness=ring_w)
    audit("버티포트 외곽 지름", 2 * r_outer / PX_PER_M, VERTIPORT_D)

    # V 픽토그램 - 안전구역 링 안쪽에 들어가도록 (안전구역 반경 기준으로 스케일)
    v_half_w = int(r_inner * 0.62)
    v_top_y  = oy - int(r_inner * 0.66)
    v_bot_y  = oy + int(r_inner * 0.72)
    v_thick  = max(4, m2px(0.22))
    cv2.polylines(img, [np.array([[ox - v_half_w, v_top_y], [ox, v_bot_y]], np.int32)],
                  False, COL_V_LOGO, v_thick, cv2.LINE_AA)
    cv2.polylines(img, [np.array([[ox + v_half_w, v_top_y], [ox, v_bot_y]], np.int32)],
                  False, COL_V_LOGO, v_thick, cv2.LINE_AA)

    # 안전구역 링 (지름 2 m) - V 위에 그려 가려지지 않게
    cv2.circle(img, (ox, oy), r_inner, COL_SAFEZONE_RING, thickness=zone_w)
    audit("항기 안전구역 지름", 2 * r_inner / PX_PER_M, SAFEZONE_D)

    if with_aruco:
        # 중앙 ArUco (ID 0, 50x50 cm) - 흰 여백 위에 얹어 V 픽토그램과 분리
        half = m2px(MARKER_SIZE / 2)
        pad  = m2px(0.04)
        cv2.rectangle(img, (ox - half - pad, oy - half - pad),
                           (ox + half + pad, oy + half + pad), (255, 255, 255), -1)
        draw_aruco(img, origin_m, cx, cy, MARKER_SIZE, 0)


# =======================================================================
#  1) 바닥 텍스처 (safe area 전체 32 x 23 m)
# =======================================================================
W_PX, H_PX = m2px(SAFE_W), m2px(SAFE_H)
GROUND_ORIGIN = (-SAFE_W / 2, SAFE_H / 2)     # 이미지 좌상단 = 월드 (-16, +11.5)

img = np.full((H_PX, W_PX, 3), COL_SAFE_BG, dtype=np.uint8)
audit("Safe area 가로", W_PX / PX_PER_M, SAFE_W)
audit("Safe area 세로", H_PX / PX_PER_M, SAFE_H)

def g2i(x, y):
    """월드 좌표 -> 바닥 텍스처 픽셀"""
    return m2px(x - GROUND_ORIGIN[0]), m2px(GROUND_ORIGIN[1] - y)

# --- Mission area 배경 ---
mx0, my0 = g2i(-MISSION_W / 2,  MISSION_H / 2)
mx1, my1 = g2i( MISSION_W / 2, -MISSION_H / 2)
cv2.rectangle(img, (mx0, my0), (mx1, my1), COL_MISSION_BG, -1)
audit("Mission area 가로", (mx1 - mx0) / PX_PER_M, MISSION_W)
audit("Mission area 세로", (my1 - my0) / PX_PER_M, MISSION_H)
audit("Mission->Safe 여유(좌)", (mx0 - 0) / PX_PER_M, 4.0)
audit("Mission->Safe 여유(하)", (H_PX - my1) / PX_PER_M, 4.0)

# --- 격자 구획선 (3 m 간격, 폭 10 cm) ---
half_lw = LINE_W / 2
n_cols = int(round(MISSION_W / GRID))   # 8
n_rows = int(round(MISSION_H / GRID))   # 5

for i in range(n_cols + 1):                       # 세로선
    x = -MISSION_W / 2 + i * GRID
    p0 = g2i(x - half_lw,  MISSION_H / 2)
    p1 = g2i(x + half_lw, -MISSION_H / 2)
    cv2.rectangle(img, p0, p1, COL_GRID_LINE, -1)
audit("구획선 폭", (g2i(half_lw, 0)[0] - g2i(-half_lw, 0)[0]) / PX_PER_M, LINE_W)
audit("격자 간격(가로)", (g2i(GRID, 0)[0] - g2i(0, 0)[0]) / PX_PER_M, GRID)

for j in range(n_rows + 1):                       # 가로선
    y = -MISSION_H / 2 + j * GRID
    p0 = g2i(-MISSION_W / 2, y + half_lw)
    p1 = g2i( MISSION_W / 2, y - half_lw)
    cv2.rectangle(img, p0, p1, COL_GRID_LINE, -1)
audit("격자 간격(세로)", (g2i(0, 0)[1] - g2i(0, GRID)[1]) / PX_PER_M, GRID)

# --- 격자 교차점 28개 ---
xs = [-MISSION_W / 2 + i * GRID for i in range(1, n_cols)]   # 7개
ys = [-MISSION_H / 2 + j * GRID for j in range(1, n_rows)]   # 4개
all_intersections = [(x, y) for x in xs for y in ys]         # 28개
assert len(all_intersections) == 28

# --- 채점 대상 경로점 4개 (ID 1~4) ---
waypoints = [
    {"id": 1, "x": -6.0, "y": -4.5},
    {"id": 2, "x": -3.0, "y":  4.5},
    {"id": 3, "x":  3.0, "y": -1.5},
    {"id": 4, "x":  9.0, "y":  4.5},
]
for wp in waypoints:
    assert (wp["x"], wp["y"]) in all_intersections, f"경로점이 교차점 위가 아님: {wp}"
    draw_aruco(img, GROUND_ORIGIN, wp["x"], wp["y"], MARKER_SIZE, wp["id"])

# --- 버티포트 위치: 규정집 다이어그램 기준, mission area 남서쪽 모서리 바깥
#     대각선 여유대 포켓(가로 4m x 세로 4m) 정중앙. 다이어그램에 "4m"가 가로/세로
#     양쪽에 표기되어 있어 코너 배치이며, 마진 폭(4m)의 절반씩 오프셋을 주면
#     mission area/safe area 양쪽에 0.5m씩 대칭 여유가 생긴다. ---
MARGIN = SAFE_W / 2 - MISSION_W / 2   # 4.0 m 여유대 폭 (가로/세로 동일)
vertiport = {"x": -(MISSION_W / 2 + MARGIN / 2), "y": -(MISSION_H / 2 + MARGIN / 2)}   # (-14.0, -9.5)
# 지름 3 m 원이 여유대 안에, 그리고 safe area 안에 들어가는지 검증
assert vertiport["x"] - VERTIPORT_D/2 > -SAFE_W/2,    "버티포트가 safe area 밖으로 나감"
assert vertiport["x"] + VERTIPORT_D/2 < -MISSION_W/2, "버티포트가 mission area를 침범"
assert vertiport["y"] - VERTIPORT_D/2 > -SAFE_H/2,    "버티포트가 safe area 밖으로 나감"
assert vertiport["y"] + VERTIPORT_D/2 < -MISSION_H/2, "버티포트가 mission area를 침범"

# 바닥에는 실물 패드(높이 0.7 m)가 놓이므로 발자국(footprint)만 옅게 표시.
# 실제 V 픽토그램 + ArUco(ID0)는 패드 상판 텍스처가 담당한다.
vx, vy = g2i(vertiport["x"], vertiport["y"])
cv2.circle(img, (vx, vy), m2px(VERTIPORT_D / 2), (205, 205, 205), thickness=max(2, m2px(0.03)))

# --- 경계선 (참고용) ---
cv2.rectangle(img, (1, 1), (W_PX - 2, H_PX - 2), (200, 120, 0), max(2, m2px(0.02)))   # safe
cv2.rectangle(img, (mx0, my0), (mx1, my1), (255, 255, 255), max(2, m2px(0.02)))       # mission

os.makedirs(f"{BASE}/models/kasa_ground_field/materials/textures", exist_ok=True)
cv2.imwrite(f"{BASE}/models/kasa_ground_field/materials/textures/field_texture.png", img)


# =======================================================================
#  2) 버티포트 패드 상판 텍스처 (3.2 x 3.2 m, 동일 축척)
# =======================================================================
P_PX = m2px(PAD_TEX_M)
pad_img = np.full((P_PX, P_PX, 3), COL_SAFE_BG, dtype=np.uint8)
PAD_ORIGIN = (-PAD_TEX_M / 2, PAD_TEX_M / 2)      # 패드 중심이 (0,0)
audit("패드 텍스처 범위", P_PX / PX_PER_M, PAD_TEX_M)

draw_vertiport(pad_img, PAD_ORIGIN, 0.0, 0.0, with_aruco=True)

os.makedirs(f"{BASE}/models/vertiport_pad/materials/textures", exist_ok=True)
cv2.imwrite(f"{BASE}/models/vertiport_pad/materials/textures/vertiport_marker.png", pad_img)


# =======================================================================
#  3) 좌표계 원점 이동
# =======================================================================
# 규정: "GPS 신호가 수신되지 않는 실내 환경을 고려하여 출발점 기준으로
#        정확한 현재 위치를 계산할 수 있어야 한다"
# -> 출발점(버티포트)을 world 원점 (0,0,0) 으로 둔다.
#    이렇게 하면 PX4 EKF local origin(= 스폰 지점) / Gazebo ground truth /
#    채점 좌표계가 모두 일치하여 좌표 변환이 전혀 필요 없다.
ORIGIN_AT_VERTIPORT = True

if ORIGIN_AT_VERTIPORT:
    OFF_X, OFF_Y = -vertiport["x"], -vertiport["y"]     # (+14.0, +9.5)
    ORIGIN_DESC = "버티포트(출발점) 중심 = (0,0,0). 규정 '출발점 기준' 과 일치."
else:
    OFF_X, OFF_Y = 0.0, 0.0
    ORIGIN_DESC = "Safe area 중심 = (0,0,0)."

def W(x, y):
    """필드 로컬 좌표 -> 최종 world 좌표"""
    return round(x + OFF_X, 3), round(y + OFF_Y, 3)

vp_w      = W(vertiport["x"], vertiport["y"])          # (0, 0)
field_w   = W(0.0, 0.0)                                # 바닥 모델 배치 위치
mission_w = W(0.0, 0.0)
assert not ORIGIN_AT_VERTIPORT or vp_w == (0.0, 0.0), vp_w

# 드론 스폰: 버티포트 상판(0.7 m) 바로 위 + 착지 여유
SPAWN = (vp_w[0], vp_w[1], round(VERTIPORT_H + 0.05, 3))

# =======================================================================
#  4) 좌표/제원 저장
# =======================================================================
config = {
    "note": ORIGIN_DESC,
    "units": "meters",
    "frame": "ENU (x=East, y=North, z=Up) — Gazebo world 기준",
    "texture_scale_px_per_m": PX_PER_M,
    "origin_at_vertiport": ORIGIN_AT_VERTIPORT,
    "drone_spawn": {"x": SPAWN[0], "y": SPAWN[1], "z": SPAWN[2],
                    "note": "버티포트 상판(0.7m) 위. PX4 EKF local origin 이 여기가 됨."},
    "mission_area": {"w": MISSION_W, "h": MISSION_H, "center": list(mission_w),
                     "x_range": [round(mission_w[0]-MISSION_W/2,3), round(mission_w[0]+MISSION_W/2,3)],
                     "y_range": [round(mission_w[1]-MISSION_H/2,3), round(mission_w[1]+MISSION_H/2,3)]},
    "safe_area":    {"w": SAFE_W,    "h": SAFE_H,    "center": list(field_w),
                     "x_range": [round(field_w[0]-SAFE_W/2,3), round(field_w[0]+SAFE_W/2,3)],
                     "y_range": [round(field_w[1]-SAFE_H/2,3), round(field_w[1]+SAFE_H/2,3)]},
    "margin_mission_to_safe_m": 4.0,
    "grid": {"spacing": GRID, "line_width": LINE_W, "n_cols": n_cols, "n_rows": n_rows},
    "all_grid_intersections_28": [
        {"x": W(x, y)[0], "y": W(x, y)[1]} for x, y in all_intersections],
    "scored_waypoints_aruco": [
        {"id": wp["id"], "x": W(wp["x"], wp["y"])[0], "y": W(wp["x"], wp["y"])[1], "z": 0.0,
         "marker_size_m": MARKER_SIZE, "dictionary": "DICT_5X5_50"} for wp in waypoints
    ],
    "optimal_search_order": [wp["id"] for wp in sorted(waypoints, key=lambda w:  w["id"])],
    "rescue_return_order":  [wp["id"] for wp in sorted(waypoints, key=lambda w: -w["id"])],
    "vertiport": {
        "x": vp_w[0], "y": vp_w[1], "z": 0.0,
        "diameter_m": VERTIPORT_D,
        "height_m": VERTIPORT_H,
        "aircraft_safe_zone_diameter_m": SAFEZONE_D,
        "center_marker_size_m": MARKER_SIZE,
        "center_marker_id": 0,
        "dictionary": "DICT_5X5_50",
        "marker_top_z_m": VERTIPORT_H,
        "pad_texture_extent_m": PAD_TEX_M,
    },
}
import yaml
with open(f"{BASE}/models/kasa_ground_field/waypoints.yaml", "w") as f:
    yaml.safe_dump(config, f, allow_unicode=True, sort_keys=False)
with open(f"{BASE}/models/kasa_ground_field/waypoints.json", "w") as f:
    json.dump(config, f, ensure_ascii=False, indent=2)


# =======================================================================
#  5) world 파일 생성 (좌표가 config 와 어긋날 수 없도록 여기서 함께 출력)
# =======================================================================
WORLD = f"""<?xml version="1.0"?>
<!-- 자동 생성 파일 - 직접 수정하지 말고 scripts/generate_field.py 를 고친 뒤 재실행하세요. -->
<sdf version="1.6">
  <world name="kasa_indoor_search">

    <!-- 2026-11-04: libgazebo_ros_init.so (/clock 퍼블리셔)는 WorldPlugin이 아니라
         SystemPlugin이라 여기 <world><plugin>으로 넣으면 "incorrect plugin type"
         에러가 남 - gzserver 실행 커맨드에 `-s libgazebo_ros_init.so`로 로드해야
         함 (kasa_tarot_launch.sh 쪽에서 처리). -->

    <!-- model://sun include 대신 직접 정의 (온라인 모델 DB 조회로 인한 hang 방지) -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- 실내(공장) 임무 컨셉 - 낮은 조도 -->
    <scene>
      <ambient>0.35 0.35 0.38 1</ambient>
      <background>0.1 0.1 0.12 1</background>
      <shadows>true</shadows>
    </scene>

    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>

    <!-- ===================================================================
         좌표계: {ORIGIN_DESC}
         드론 스폰 위치 : ({SPAWN[0]}, {SPAWN[1]}, {SPAWN[2]})  <- 버티포트 상판 위
         Mission area   : x [{config['mission_area']['x_range'][0]} .. {config['mission_area']['x_range'][1]}]  y [{config['mission_area']['y_range'][0]} .. {config['mission_area']['y_range'][1]}]
         경로점(ArUco)   : {', '.join(f"ID{w['id']}({w['x']}, {w['y']})" for w in config['scored_waypoints_aruco'])}
         =================================================================== -->

    <!-- 바닥면: safe area 32x23m + mission area 24x15m + 3m 격자 + 경로점 ArUco 4개 -->
    <include>
      <uri>model://kasa_ground_field</uri>
      <pose>{field_w[0]} {field_w[1]} 0 0 0 0</pose>
    </include>

    <!-- 버티포트: 지름 3m, 높이 0.7m, 상판에 V픽토그램 + 중앙 ArUco(ID0) -->
    <include>
      <uri>model://vertiport_pad</uri>
      <pose>{vp_w[0]} {vp_w[1]} 0 0 0 0</pose>
    </include>

    <!-- 2026-11-04: tarot680pro embedded directly in the world instead of
         spawned later via `gz model --spawn-file` - the external spawn
         (after gzserver is already running) was leaving the GPS plugin's
         very first samples looking discontinuous to EKF2, which latched
         ESTIMATOR_GPS_GLITCH permanently and blocked arming no matter what
         EKF2 param was changed. Embedding it here so it's part of the
         world's state from t=0, like the standard `make px4_sitl` flow's
         empty.world does, fixed it (confirmed via isolated A/B test). -->
    <include>
      <uri>model://tarot680pro</uri>
      <pose>{SPAWN[0]} {SPAWN[1]} {SPAWN[2]} 0 0 0</pose>
    </include>

    <gui>
      <camera name="user_camera">
        <pose>{round(mission_w[0]-2,2)} {round(mission_w[1]-24,2)} 20 0 0.72 1.5708</pose>
      </camera>
    </gui>

  </world>
</sdf>
"""
os.makedirs(f"{BASE}/worlds", exist_ok=True)
with open(f"{BASE}/worlds/kasa_indoor_search.world", "w") as f:
    f.write(WORLD)


# =======================================================================
#  6) 축척 검증 리포트
# =======================================================================
print(f"\n{'항목':<24}{'실측(m)':>12}{'규정(m)':>12}   판정")
print("-" * 62)
ok = True
for name, meas, spec in _audit:
    good = abs(meas - spec) <= 0.005          # 5 mm 이내
    ok &= good
    print(f"{name:<24}{meas:>12.3f}{spec:>12.3f}   {'OK' if good else '*** 불일치 ***'}")
print("-" * 62)
print(f"바닥 텍스처      : {img.shape[1]}x{img.shape[0]} px  @ {PX_PER_M:.0f} px/m")
print(f"버티포트 텍스처   : {pad_img.shape[1]}x{pad_img.shape[0]} px  @ {PX_PER_M:.0f} px/m")
print(f"\n전체 판정: {'모든 비율 일치' if ok else '불일치 항목 있음'}")
assert ok, "축척 검증 실패"

print(f"\n=== 좌표계 ===")
print(f"원점        : {ORIGIN_DESC}")
print(f"버티포트     : ({vp_w[0]}, {vp_w[1]}, 0)   상판 높이 {VERTIPORT_H} m")
print(f"드론 스폰    : ({SPAWN[0]}, {SPAWN[1]}, {SPAWN[2]})")
print(f"Mission area: x [{config['mission_area']['x_range'][0]} .. {config['mission_area']['x_range'][1]}]"
      f"  y [{config['mission_area']['y_range'][0]} .. {config['mission_area']['y_range'][1]}]")
print(f"Safe area   : x [{config['safe_area']['x_range'][0]} .. {config['safe_area']['x_range'][1]}]"
      f"  y [{config['safe_area']['y_range'][0]} .. {config['safe_area']['y_range'][1]}]")
print("\n경로점 (탐색순서 = ID 오름차순, 복귀 = 역순)")
for w in config["scored_waypoints_aruco"]:
    d = (w["x"]**2 + w["y"]**2) ** 0.5
    print(f"  ID{w['id']}  ({w['x']:>6.2f}, {w['y']:>5.2f})   버티포트로부터 {d:5.2f} m")
print(f"\n생성: worlds/kasa_indoor_search.world")
