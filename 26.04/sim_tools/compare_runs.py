"""두 비행 ulog 비교: 궤적(EKF/실제), 위치오차, 고도 + 요약표.
사용: .venv-px4/bin/python compare_runs.py A.ulg 'A이름' B.ulg 'B이름' out.png"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as _fm
_fm.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams['font.family'] = _fm.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name()
plt.rcParams['axes.unicode_minus'] = False
from pyulog import ULog

def load(p):
    u = ULog(p, ["vehicle_local_position", "vehicle_local_position_groundtruth", "vehicle_land_detected"])
    d = {x.name: x.data for x in u.data_list}
    t0 = float(u.start_timestamp)
    T = lambda a: (a["timestamp"].astype(np.float64) - t0) / 1e6
    lp, gt, ld = d["vehicle_local_position"], d["vehicle_local_position_groundtruth"], d["vehicle_land_detected"]
    air = T(ld)[ld["landed"] == 0]; ta, tb = air[0], air[-1]
    m = (T(lp) >= ta) & (T(lp) <= tb); mg = (T(gt) >= ta) & (T(gt) <= tb)
    tl = T(lp)[m] - ta
    e = np.hypot(np.interp(T(lp)[m], T(gt), gt["x"]) - lp["x"][m], np.interp(T(lp)[m], T(gt), gt["y"]) - lp["y"][m])
    return dict(lp=lp, gt=gt, m=m, mg=mg, t=tl, e=e, alt=-lp["z"][m], dur=tb - ta)

A, na, B, nb, out = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5]
runs = [(load(A), na), (load(B), nb)]
fig = plt.figure(figsize=(15, 11))
g = fig.add_gridspec(2, 2, height_ratios=[1.6, 1])
for k, (r, name) in enumerate(runs):
    ax = fig.add_subplot(g[0, k])
    for x in range(2, 27, 3): ax.axvline(x, color="0.88", lw=.8, zorder=0)
    for y in range(2, 18, 3): ax.axhline(y, color="0.88", lw=.8, zorder=0)
    ax.add_patch(plt.Circle((0, 0), 1.5, color="orange", alpha=.3))
    for (mx, my) in [(8, 5), (11, 14), (17, 8), (23, 14)]: ax.plot(mx, my, "ks", ms=7)
    ax.plot(r["lp"]["y"][r["m"]], r["lp"]["x"][r["m"]], lw=1.5, label="EKF")
    ax.plot(r["gt"]["y"][r["mg"]], r["gt"]["x"][r["mg"]], "--", lw=1.1, label="truth")
    ax.set_aspect("equal"); ax.set_xlim(-3, 27); ax.set_ylim(-3, 18)
    ax.set_title(f"{name}\n오차 평균 {r['e'].mean():.2f} m / 95% {np.percentile(r['e'],95):.2f} m / 최대 {r['e'].max():.2f} m, 비행 {r['dur']:.0f}s")
    ax.legend(loc="upper left", fontsize=8)
ax = fig.add_subplot(g[1, 0])
for r, name in runs: ax.plot(r["t"], r["e"], lw=1, label=name)
ax.set_xlabel("이륙 후 시간 [s]"); ax.set_ylabel("|EKF - truth| [m]"); ax.legend(); ax.grid(alpha=.3)
ax = fig.add_subplot(g[1, 1])
for r, name in runs: ax.plot(r["t"], r["alt"], lw=1, label=name)
ax.set_xlabel("이륙 후 시간 [s]"); ax.set_ylabel("EKF 고도(상판 기준) [m]"); ax.legend(); ax.grid(alpha=.3)
plt.tight_layout(); plt.savefig(out, dpi=85); print("saved", out)
for r, name in runs:
    print(f"{name}: dur {r['dur']:.0f}s, err mean {r['e'].mean():.3f} p95 {np.percentile(r['e'],95):.3f} max {r['e'].max():.3f}")
