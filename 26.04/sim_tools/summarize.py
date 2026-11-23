"""results.csv -> SUMMARY.md + summary.png (설정별 비교). 매 회차 뒤 자동 실행."""
import csv, sys, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
FP = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
fm.fontManager.addfont(FP); plt.rcParams["font.family"] = fm.FontProperties(fname=FP).get_name()
plt.rcParams["axes.unicode_minus"] = False
D = sys.argv[1]
rows = list(csv.DictReader(open(f"{D}/results.csv")))
NAMES = {"A": "A: 전방+하향, GPS 켬", "B": "B: 하향만, GPS 켬", "C": "C: 전방+하향, GPS 끔(라인EV)"}
f = lambda r, k: float(r[k]) if r.get(k) not in (None, "") else np.nan
lines = ["# 밤샘 배치 결과 (자동 생성)", "", f"총 {len(rows)}회. 마커 위치는 매 회차 28개 교차점 중 무작위 4곳.", "",
         "| 설정 | 회수 | 4/4 정답 | 마커 정답률 | 오등록 | 평균 비행시간 | EKF오차 최대(평균) | 필드이탈 | 노드완료 | 안전장치 작동 |",
         "|---|---|---|---|---|---|---|---|---|---|"]
for c in "ABC":
    R = [r for r in rows if r["config"] == c]
    if not R:
        continue
    n = len(R)
    full = sum(1 for r in R if r["correct"] == "4")
    acc = np.mean([f(r, "correct") / 4 for r in R]) * 100
    wrong = sum(int(r["wrong"]) for r in R)
    fl = np.nanmean([f(r, "flight_s") for r in R])
    em = np.nanmean([f(r, "ekf_err_max") for r in R])
    out = sum(1 for r in R if f(r, "truth_max_out") > 0.5)
    done = sum(int(r["done"]) for r in R)
    safe = sum(1 for r in R if r["watchdog"] == "1" or r["guard"] == "1" or r["timeout_kill"] == "1")
    lines.append(f"| {NAMES[c]} | {n} | {full}/{n} | {acc:.0f}% | {wrong} | {fl:.0f}s | {em:.2f} m | {out} | {done}/{n} | {safe} |")
lines += ["", "열 설명: 4/4 정답 = 마커 4개 모두 맞는 칸에 기록, 오등록 = 틀린 칸에 기록된 마커 수, 필드이탈 = 실제 위치가 안전구역 밖 0.5m 이상,",
          "안전장치 = 라인위치 끊김 착륙 / 실제위치 감시 착륙 / 10분 시간초과 중 하나.", "", "## 회차별", "",
          "| run | 설정 | 마커(ID:x:y) | 정답 | 오등록 | 전방등록(정답) | 탐색칸 | 비행s | EKF오차max | 이탈m | 비고 |", "|---|---|---|---|---|---|---|---|---|---|---|"]
for r in rows:
    note = " ".join(k for k in ["watchdog", "guard", "timeout_kill", "traceback"] if r[k] == "1") or ("" if r["done"] == "1" else "미완료")
    lines.append(f"| {r['run']} | {r['config']} | {r['markers']} | {r['correct']} | {r['wrong']} | {r['front_reg']}({r['front_ok']}) | "
                 f"{r['search_cells']} | {r['flight_s']} | {r['ekf_err_max']} | {r['truth_max_out']} | {note} |")
open(f"{D}/SUMMARY.md", "w").write("\n".join(lines) + "\n")

fig, ax = plt.subplots(1, 3, figsize=(16, 5))
for k, c in enumerate("ABC"):
    R = [r for r in rows if r["config"] == c]
    if not R:
        continue
    x = k + np.random.uniform(-0.12, 0.12, len(R))
    ax[0].scatter(x, [f(r, "correct") for r in R], alpha=.6)
    ax[1].scatter(x, [f(r, "flight_s") for r in R], alpha=.6)
    ax[2].scatter(x, [f(r, "ekf_err_max") for r in R], alpha=.6)
for a, t in zip(ax, ["정답 마커 수 (/4)", "비행 시간 [s]", "EKF-실제 최대오차 [m] (log)"]):
    a.set_xticks(range(3)); a.set_xticklabels([NAMES[c].replace(", ", "\n") for c in "ABC"], fontsize=8); a.set_title(t); a.grid(alpha=.3)
ax[2].set_yscale("log")
plt.tight_layout(); plt.savefig(f"{D}/summary.png", dpi=80)
print("summary updated", len(rows))
