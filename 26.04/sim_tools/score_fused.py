"""fused_line_node CSV 채점: 하향/전방/융합 lat·psi를 gz 실제 위치·기수로 계산한 참값과 비교.
시간 정렬은 score_arrivals.py와 같은 방식 (CSV ekf_E/N을 ulog EKF 궤적에 맞춤).
사용: .venv-px4/bin/python score_fused.py fused.csv run.ulg [out.png]"""
import csv
import math
import sys

import numpy as np
from pyulog import ULog

X_LINES = [-2.0] + [2.0 + 3.0 * i for i in range(9)] + [30.0]   # East 일정 (남북 선)
Y_LINES = [-2.0] + [2.0 + 3.0 * j for j in range(6)] + [21.0]   # North 일정 (동서 선)

rows = list(csv.DictReader(open(sys.argv[1])))
d = {x.name: x.data for x in ULog(sys.argv[2], ["vehicle_local_position_groundtruth", "vehicle_local_position",
                                                "vehicle_attitude_groundtruth"]).data_list}
gt, lp = d["vehicle_local_position_groundtruth"], d["vehicle_local_position"]
tg, tl = gt["timestamp"].astype(float), lp["timestamp"].astype(float)
t = np.array([float(r["px4_t"]) for r in rows])
ekf = np.array([[float(r["ekf_E"]), float(r["ekf_N"])] for r in rows])
sel = np.linspace(0, len(t) - 1, min(400, len(t))).astype(int)


def mis(off):
    E, N = np.interp(t[sel] - off, tl, lp["y"]), np.interp(t[sel] - off, tl, lp["x"])
    return np.mean(np.hypot(E - ekf[sel, 0], N - ekf[sel, 1]))


o0 = t[0] - tl[0]
b = min(np.arange(o0 - 1500e6, o0 + 1500e6, 0.5e6), key=mis)
b = min(np.arange(b - 1e6, b + 1e6, 0.01e6), key=mis)
print(f"시간 정렬 잔차 {mis(b) * 100:.1f} cm")
E = np.interp(t - b, tg, gt["y"]); N = np.interp(t - b, tg, gt["x"])
if "vehicle_attitude_groundtruth" in d:
    at = d["vehicle_attitude_groundtruth"]; ta = at["timestamp"].astype(float)
    q0, q1, q2, q3 = (np.interp(t - b, ta, at[f"q[{i}]"]) for i in range(4))
    yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 ** 2 + q3 ** 2))
else:
    yaw = np.interp(t - b, tg, gt["heading"])
hgt = -np.interp(t - b, tg, gt["z"])


def truth(i, lat_meas):
    """진행방향(기수)에 가까운 축의 격자선 중 측정 lat에 가장 가까운 선의 참 lat, 참 psi."""
    c, s = math.cos(yaw[i]), math.sin(yaw[i])          # 전방 (N,E) = (c,s), 오른쪽 = (-s,c)
    if abs(s) > abs(c):     # 동/서 진행 -> 동서 선 (N 일정)
        rn = -s
        cands = [(Nl - N[i]) / rn for Nl in Y_LINES]
        axis = math.pi / 2 if s > 0 else -math.pi / 2
    else:                   # 남/북 진행 -> 남북 선 (E 일정)
        re_ = c
        cands = [(El - E[i]) / re_ for El in X_LINES]
        axis = 0.0 if c > 0 else math.pi
    psi = (axis - yaw[i] + math.pi) % (2 * math.pi) - math.pi
    psi = (psi + math.pi / 2) % math.pi - math.pi / 2
    lt = min(cands, key=lambda v: abs(v - lat_meas))
    return lt, psi


inside = (E > 3) & (E < 25) & (N > 3) & (N < 16)
print(f"구역 안 프레임 {inside.sum()} / {len(t)}, 실제 지면고도 중앙 {np.median(hgt[inside]):.2f} m (CSV h 중앙 "
      f"{np.median([float(rows[i]['h']) for i in np.where(inside)[0]]):.2f})")
res = {}
for key in ("d", "f", "u"):
    el, ep, nl = [], [], 0
    for i in np.where(inside)[0]:
        v = rows[i][f"lat_{key}"]
        if v == "":
            continue
        nl += 1
        lt, ps = truth(i, float(v))
        el.append(float(v) - lt)
        if rows[i][f"psi_{key}"] != "":
            ep.append(float(rows[i][f"psi_{key}"]) - ps)
    el, ep = np.array(el), np.degrees(np.array(ep))
    res[key] = (el, ep)
    nm = {"d": "하향", "f": "전방", "u": "융합"}[key]
    if len(el):
        print(f"  {nm}: 검출 {nl / inside.sum() * 100:3.0f}% | lat 오차 중앙 {np.median(el):+.3f} m, |오차| 중앙 "
              f"{np.median(abs(el)):.3f}, 95% {np.percentile(abs(el), 95):.3f}, >0.3m {np.mean(abs(el) > 0.3) * 100:.1f}%"
              + (f" | psi 오차 중앙 {np.median(ep):+.2f}°, |오차| 95% {np.percentile(abs(ep), 95):.2f}° (n={len(ep)})"
                 if len(ep) else " | psi 없음"))
    else:
        print(f"  {nm}: 검출 0")
ms = np.array([float(r["proc_ms"]) for r in rows])
print(f"처리시간 중앙 {np.median(ms):.1f} ms, 95% {np.percentile(ms, 95):.1f} ms")
if len(sys.argv) > 3:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for key, c in (("d", "C0"), ("f", "C1"), ("u", "C2")):
        el, ep = res[key]
        if len(el): ax[0].hist(np.clip(el, -0.6, 0.6), 60, alpha=.5, color=c, label=key)
        if len(ep): ax[1].hist(np.clip(ep, -10, 10), 60, alpha=.5, color=c, label=key)
    ax[0].set_xlabel("lat error [m]"); ax[1].set_xlabel("psi error [deg]")
    for a in ax: a.legend()
    plt.tight_layout(); plt.savefig(sys.argv[3], dpi=110)
