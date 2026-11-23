"""도착 판정 채점 비교 그림: 회차별/판정원별 실제 교차점까지 거리 + 진행방향 오차."""
import sys, re, subprocess
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
FP = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
fm.fontManager.addfont(FP); plt.rcParams["font.family"] = fm.FontProperties(fname=FP).get_name()
plt.rcParams["axes.unicode_minus"] = False
sys.path.insert(0, sys.argv[1])
D = sys.argv[1]; out = sys.argv[2]
runs = [("13회차\n기존(legacy)", "run13"), ("14회차\n격자선 1프레임", "run14"), ("16회차\n격자선 5프레임 중앙값", "run16")]
import importlib.util, io, contextlib
from pyulog import ULog
rx = re.compile(r"\[ARRIVE\] src=(\w+) cell=\((\d+),(\d+)\) target=\(([\d.]+),([\d.]+)\) px4_t=(\d+) ekf=\(([-\d.]+),([-\d.]+)\)")
def score(r):
    ev = [m.groups() for m in map(rx.search, open(f"{D}/{r}_mission_node.log", encoding="utf-8", errors="ignore")) if m]
    d = {x.name: x.data for x in ULog(f"{D}/{r}.ulg", ["vehicle_local_position_groundtruth", "vehicle_local_position"]).data_list}
    gt, lp = d["vehicle_local_position_groundtruth"], d["vehicle_local_position"]
    tg, tl = gt["timestamp"].astype(float), lp["timestamp"].astype(float)
    t = np.array([float(e[5]) for e in ev]); ekf = np.array([[float(e[6]), float(e[7])] for e in ev])
    tgt = np.array([[float(e[3]), float(e[4])] for e in ev]); src = np.array([e[0] for e in ev])
    mis = lambda o: np.mean(np.hypot(np.interp(t - o, tl, lp["y"]) - ekf[:, 0], np.interp(t - o, tl, lp["x"]) - ekf[:, 1]))
    o = t[0] - tl[0]; o = min(np.arange(o - 400e6, o + 400e6, 0.5e6), key=mis); o = min(np.arange(o - 1e6, o + 1e6, 1e4), key=mis)
    E, N = np.interp(t - o, tg, gt["y"]), np.interp(t - o, tg, gt["x"])
    vE, vN = np.interp(t - o, tg, gt["vy"]), np.interp(t - o, tg, gt["vx"])
    err = np.hypot(E - tgt[:, 0], N - tgt[:, 1])
    along = ((E - tgt[:, 0]) * vE + (N - tgt[:, 1]) * vN) / np.maximum(np.hypot(vE, vN), 1e-3)
    return src, err, along
S = [(lab, *score(r)) for lab, r in runs]
fig, ax = plt.subplots(1, 3, figsize=(16, 6))
col = {"legacy": "tab:red", "grid": "tab:green", "ekf": "tab:gray"}
for k, (lab, src, err, along) in enumerate(S):
    for s in ["ekf", "legacy", "grid"]:
        m = src == s
        if m.any():
            x = k + {"ekf": 0.2, "legacy": -0.2, "grid": -0.2}[s] + np.random.uniform(-0.07, 0.07, m.sum())
            ax[0].scatter(x, err[m], s=18, color=col[s], alpha=.7, label=s if k == 0 or (s == "grid" and k == 1) else None)
            ax[1].scatter(x, along[m], s=18, color=col[s], alpha=.7)
ax[0].axhline(0.5, ls="--", c="k", lw=.8); ax[0].set_ylabel("판정 순간 실제 위치 ↔ 교차점 거리 [m]")
ax[1].axhline(0, c="k", lw=.8); ax[1].set_ylabel("진행방향 오차 [m]  (− = 교차점 전에 판정)")
for a in ax[:2]:
    a.set_xticks(range(3)); a.set_xticklabels([s[0] for s in S]); a.grid(alpha=.3)
ax[0].legend(title="판정원", loc="upper right"); ax[0].set_title("판정 정확도 (점 1개 = 도착 판정 1회)")
ax[1].set_title("일찍/늦게 판정했나")
# 표
rows = []
for lab, src, err, along in S:
    for s in ["legacy", "grid", "ekf"]:
        m = src == s
        if m.any():
            rows.append([lab.replace("\n", " "), s, f"{m.sum()}/{len(src)}", f"{err[m].mean():.2f}", f"{np.mean(err[m]<=0.5)*100:.0f}%", f"{along[m].mean():+.2f}"])
ax[2].axis("off")
tb = ax[2].table(cellText=rows, colLabels=["회차", "판정원", "횟수", "평균거리 m", "≤0.5m", "진행방향 m"], loc="center", cellLoc="center")
tb.auto_set_font_size(False); tb.set_fontsize(9); tb.scale(1, 1.6)
ax[2].set_title("요약 (모두 GPS 켬, EKF 동일)")
plt.tight_layout(); plt.savefig(out, dpi=85); print("saved", out)
