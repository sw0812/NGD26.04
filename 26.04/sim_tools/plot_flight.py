"""PX4 ulog 비행 요약 그림 (KASA gz SITL).
사용: .venv-px4/bin/python plot_flight.py <log.ulg> <out.png> [제목]
  ① 위에서 본 궤적: EKF vs 실제(gz groundtruth) + 격자선 + 마커/버티포트
  ② EKF - 실제 위치 오차
  ③ 고도(지면 기준 라이다, EKF z)
  ④ EKF 융합 상태 (GPS / 라인 EV / 플로우) + 비행모드
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as _fm
_FP = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
_fm.fontManager.addfont(_FP); plt.rcParams["font.family"] = _fm.FontProperties(fname=_FP).get_name(); plt.rcParams["axes.unicode_minus"] = False
from pyulog import ULog

log, out = sys.argv[1], sys.argv[2]
title = sys.argv[3] if len(sys.argv) > 3 else log.split("/")[-1]
# 4번째 인자: 이번 회차 마커 "ID:x:y ..." (없으면 예전 고정 배치)
MARKERS = {int(i): (float(x), float(y)) for i, x, y in (t.split(":") for t in sys.argv[4].split())} if len(sys.argv) > 4 \
    else {1: (8, 5), 2: (11, 14), 3: (17, 8), 4: (23, 14)}
u = ULog(log, ["vehicle_local_position", "vehicle_local_position_groundtruth",
               "estimator_status_flags", "vehicle_status", "vehicle_land_detected"])
d = {x.name: x.data for x in u.data_list}
t0 = float(u.start_timestamp)
T = lambda a: (a["timestamp"].astype(np.float64) - t0) / 1e6

lp, gt = d["vehicle_local_position"], d.get("vehicle_local_position_groundtruth")
fl, st = d.get("estimator_status_flags"), d.get("vehicle_status")

# 비행 구간(landed=0)만
ld = d.get("vehicle_land_detected")
if ld is not None and (ld["landed"] == 0).any():
    air = T(ld)[ld["landed"] == 0]
    ta, tb = air[0] - 5, air[-1] + 5
else:
    ta, tb = T(lp)[0], T(lp)[-1]
w = lambda a: (T(a) >= ta) & (T(a) <= tb)

fig = plt.figure(figsize=(14, 12))
g = fig.add_gridspec(3, 2, height_ratios=[2.2, 1, 1])
ax0 = fig.add_subplot(g[0, :])
for x in [-2] + list(range(2, 27, 3)) + [30]:
    ax0.axvline(x, color="0.85", lw=0.8, zorder=0)
for y in [-2] + list(range(2, 18, 3)) + [21]:
    ax0.axhline(y, color="0.85", lw=0.8, zorder=0)
ax0.add_patch(plt.Circle((0, 0), 1.5, color="orange", alpha=.3, label="vertiport"))
for mid, (mx, my) in MARKERS.items():
    ax0.plot(mx, my, "ks", ms=8)
    ax0.annotate(f"ID{mid}", (mx, my), xytext=(5, 5), textcoords="offset points")
m = w(lp)
ax0.plot(lp["y"][m], lp["x"][m], label="EKF", lw=1.6)
if gt is not None:
    mg = w(gt)
    ax0.plot(gt["y"][mg], gt["x"][mg], "--", label="ground truth (gz)", lw=1.2)
ax0.set_aspect("equal")
ax0.set_xlabel("world x = East [m]")
ax0.set_ylabel("world y = North [m]")
ax0.set_title(title)
ax0.legend(loc="upper left")

ax1 = fig.add_subplot(g[1, 0])
if gt is not None:
    tl = T(lp)[m]
    e = np.hypot(np.interp(tl, T(gt), gt["x"]) - lp["x"][m], np.interp(tl, T(gt), gt["y"]) - lp["y"][m])
    ax1.plot(tl, e, "r")
    ax1.set_title(f"|EKF - truth|  mean {e.mean():.2f} m, max {e.max():.2f} m")
ax1.set_ylabel("m"); ax1.grid(alpha=.3)

ax2 = fig.add_subplot(g[1, 1], sharex=ax1)
ax2.plot(T(lp)[m], -lp["z"][m], label="EKF alt (home = pad top)")
ok = m & lp["dist_bottom_valid"].astype(bool)
ax2.plot(T(lp)[ok], lp["dist_bottom"][ok], ".", ms=1, label="lidar dist_bottom")
ax2.set_ylabel("m"); ax2.legend(fontsize=8); ax2.grid(alpha=.3); ax2.set_title("altitude")

ax3 = fig.add_subplot(g[2, :], sharex=ax1)
if fl is not None:
    mf = w(fl)
    for i, (k, c, lab) in enumerate([("cs_gnss_pos", "k", "GPS"), ("cs_ev_pos", "g", "line EV"),
                                     ("cs_opt_flow", "b", "optical flow")]):
        ax3.fill_between(T(fl)[mf], i, i + 0.8 * fl[k][mf], step="post", color=c, alpha=.6)
    ax3.set_yticks([0.4, 1.4, 2.4]); ax3.set_yticklabels(["GPS", "line EV", "flow"])
if st is not None:
    ms = w(st)
    ax3b = ax3.twinx()
    ax3b.step(T(st)[ms], st["nav_state"][ms], "m", where="post", lw=1)
    ax3b.set_ylabel("nav_state (14=OFFBOARD,18=LAND,5=RTL)", color="m", fontsize=8)
ax3.set_xlabel("time [s]"); ax3.set_title("EKF fusion / flight mode")
ax3.set_xlim(ta, tb)
plt.tight_layout()
plt.savefig(out, dpi=85)
print("saved", out)
