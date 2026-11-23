"""run8b(GPS 끔 + 라인 EV) flyaway 분석 그림: EKF 위치 vs 실제(gz groundtruth), EV/플로우 융합 상태."""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pyulog import ULog

u = ULog(sys.argv[1])
out = sys.argv[2]
d = {}
for x in u.data_list:
    d.setdefault(x.name, x.data)
t0 = u.start_timestamp


def T(a):
    return (a["timestamp"].astype(np.float64) - float(t0)) / 1e6


lp = d["vehicle_local_position"]
gt = d.get("vehicle_local_position_groundtruth")
fl = d.get("estimator_status_flags")
ev = d.get("estimator_aid_src_ev_pos")
of = d.get("estimator_aid_src_optical_flow")
st = d.get("vehicle_status")

fig, ax = plt.subplots(4, 1, figsize=(12, 13), sharex=True)
for k, (axis, name) in enumerate([("y", "East (world x)"), ("x", "North (world y)")]):
    ax[k].plot(T(lp), lp[axis], label="EKF", lw=1.5)
    if gt is not None:
        ax[k].plot(T(gt), gt[axis], label="ground truth (gz)", lw=1.5)
    ax[k].set_ylabel(f"{name} [m]")
    ax[k].legend(loc="upper left")
    ax[k].grid(alpha=.3)
ax[0].set_title("run8b: GPS off, line-EV + optical flow")

if gt is not None:
    e = np.hypot(np.interp(T(lp), T(gt), gt["x"]) - lp["x"], np.interp(T(lp), T(gt), gt["y"]) - lp["y"])
    ax[2].plot(T(lp), e, color="r")
    ax[2].set_ylabel("|EKF - truth| [m]")
    ax[2].set_yscale("symlog", linthresh=1)
    ax[2].grid(alpha=.3)

if fl is not None:
    for i, (k, c) in enumerate([("cs_gnss_pos", "k"), ("cs_ev_pos", "g"), ("cs_opt_flow", "b")]):
        if k in fl:
            ax[3].fill_between(T(fl), i, i + 0.8 * fl[k], step="post", color=c, alpha=.6, label=k)
    if ev is not None and "innovation_rejected" in ev:
        r = ev["innovation_rejected"].astype(bool)
        ax[3].plot(T(ev)[r], np.full(r.sum(), 3.4), "rx", ms=3, label="EV rejected")
    ax[3].set_yticks([0.4, 1.4, 2.4, 3.4])
    ax[3].set_yticklabels(["GPS", "EV(line)", "flow", "EV rej"])
    ax[3].legend(loc="upper left", fontsize=8)
ax[3].set_xlabel("time since log start [s]")
# GPS 끈 뒤 구간만 보기: cs_gnss_pos 마지막 True 시점 -20s ~ 끝
if fl is not None:
    g = fl["cs_gnss_pos"] > 0
    tf = T(fl)
    off = np.where(~g)[0]
    blk_end = off[-1]                      # 마지막 GPS-off 구간
    blk_start = blk_end
    while blk_start > 0 and not g[blk_start - 1]:
        blk_start -= 1
    tg, te = tf[blk_start], tf[blk_end]
    for a_ in ax:
        a_.set_xlim(tg - 20, te + 15)
    print(f"GPS off {tg:.1f}s ~ {te:.1f}s")
for k, axis in [(0, "y"), (1, "x")]:
    w = (T(lp) > tg - 20) & (T(lp) < te + 15)
    vals = [lp[axis][w]] + ([gt[axis][(T(gt) > tg - 20) & (T(gt) < te + 15)]] if gt is not None else [])
    v = np.concatenate(vals); ax[k].set_ylim(v.min() - 2, v.max() + 2)
plt.tight_layout()
plt.savefig(out, dpi=90)
print("saved", out)

# 첫 크게 벌어진 시점
if gt is not None:
    tt = T(lp); m = (tt > tg) & (tt < te)
    i = np.where(m & (e > 2.0))[0][0]
    print(f"EKF-truth > 2 m first at t={tt[i]:.1f}s ; max {e[m].max():.1f} m")
