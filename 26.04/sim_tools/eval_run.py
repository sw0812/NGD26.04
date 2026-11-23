"""배치 1회 채점 -> results.csv 한 줄 추가.
사용: python eval_run.py <run_dir> <config> <markers "1:8:5 2:..."> <results.csv>"""
import csv, os, re, sys
import numpy as np
from pyulog import ULog

run_dir, config, mk, out = sys.argv[1:5]
truth = {int(i): (float(x), float(y)) for i, x, y in (s.split(":") for s in mk.split())}
log = open(f"{run_dir}/mission_node.log", encoding="utf-8", errors="ignore").read()

found = {int(m[0]): (int(m[1]), int(m[2])) for m in re.findall(r"\[SUMMARY\] ID(\d+): grid\((\d+),(\d+)\)", log)}
X = [5.0, 8.0, 11.0, 14.0, 17.0, 20.0, 23.0]; Y = [5.0, 8.0, 11.0, 14.0]
correct = sum(1 for i, c in found.items() if i in truth and (X[c[0]], Y[c[1]]) == truth[i])
wrong = len(found) - correct
front = re.findall(r"\[FRONT\] ArUco ID(\d+) 선등록 @ grid\((\d+), (\d+)\)", log)
front_ok = sum(1 for i, a, b in front if int(i) in truth and (X[int(a)], Y[int(b)]) == truth[int(i)])

row = dict(run=os.path.basename(run_dir), config=config, markers=mk,
           found=len(found), correct=correct, wrong=wrong,
           front_reg=len(front), front_ok=front_ok,
           search_cells=log.count("[SEARCH] 교차점 도달"),
           arr_grid=log.count("src=grid"), arr_legacy=log.count("src=legacy"), arr_ekf=log.count("src=ekf"),
           confirm_timeout=log.count("타임아웃"), watchdog=int("[WATCHDOG]" in log),
           done=int("[DONE]" in log), traceback=int("Traceback" in log),
           guard=int(os.path.exists(f"{run_dir}/guard.log") and "GUARD" in open(f"{run_dir}/guard.log").read()),
           timeout_kill=int(os.path.exists(f"{run_dir}/TIMEOUT")))
try:
    u = ULog(f"{run_dir}/run.ulg", ["vehicle_local_position", "vehicle_local_position_groundtruth", "vehicle_land_detected"])
    d = {x.name: x.data for x in u.data_list}
    lp, gt, ld = d["vehicle_local_position"], d["vehicle_local_position_groundtruth"], d["vehicle_land_detected"]
    tl, tg, tld = (a["timestamp"].astype(float) for a in (lp, gt, ld))
    air = tld[ld["landed"] == 0]
    ta, tb = (air[0], air[-1]) if len(air) else (tl[0], tl[-1])
    m = (tl >= ta) & (tl <= tb)
    e = np.hypot(np.interp(tl[m], tg, gt["x"]) - lp["x"][m], np.interp(tl[m], tg, gt["y"]) - lp["y"][m])
    mg = (tg >= ta) & (tg <= tb)
    E, N = gt["y"][mg], gt["x"][mg]
    row.update(flight_s=round((tb - ta) / 1e6, 1), ekf_err_mean=round(float(e.mean()), 3), ekf_err_max=round(float(e.max()), 3),
               truth_max_out=round(float(max(0, (-E).max() - 2, (E - 30).max(), (-N).max() - 2, (N - 21).max())), 2),
               end_dist_pad=round(float(np.hypot(gt["y"][-1], gt["x"][-1])), 2),
               alt_mean=round(float(-lp["z"][m].mean()), 2), alt_max=round(float(-lp["z"][m].min()), 2))
except Exception as ex:   # ulog 없음/손상
    row.update(flight_s="", ekf_err_mean="", ekf_err_max="", truth_max_out="", end_dist_pad="", alt_mean="", alt_max="", ulog_err=str(ex)[:80])
new = not os.path.exists(out)
keys = ["run", "config", "markers", "found", "correct", "wrong", "front_reg", "front_ok", "search_cells", "arr_grid", "arr_legacy",
        "arr_ekf", "confirm_timeout", "watchdog", "guard", "timeout_kill", "done", "traceback", "flight_s", "ekf_err_mean",
        "ekf_err_max", "truth_max_out", "end_dist_pad", "alt_mean", "alt_max", "ulog_err"]
with open(out, "a", newline="") as f:
    w = csv.DictWriter(f, fieldnames=keys)
    if new:
        w.writeheader()
    w.writerow({k: row.get(k, "") for k in keys})
print({k: row.get(k) for k in ["run", "config", "correct", "wrong", "flight_s", "ekf_err_max", "truth_max_out", "done"]})
