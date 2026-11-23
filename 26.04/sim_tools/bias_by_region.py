"""line_odom CSV(pn,pe 포함): 위치 구간별 라인-EKF 편향."""
import sys, numpy as np
d = np.genfromtxt(sys.argv[1], delimiter=',', names=True)
r = np.hypot(d['pn'], d['pe'])
for lo, hi in [(0, 4), (4, 7), (7, 40)]:
    m = (r >= lo) & (r < hi)
    for ax in ('err_n', 'err_e'):
        v = d[ax][m]; v = v[~np.isnan(v)]
        if len(v):
            print(f"dist {lo:2d}-{hi:2d}m {ax}: n={len(v):4d} mean={v.mean():+.3f} std={v.std():.3f} max|{np.abs(v).max():.2f}|")
for ax in ('err_n', 'err_e'):
    v = d[ax][~np.isnan(d[ax])]
    print(f"ALL {ax}: n={len(v)} rms={np.sqrt(np.mean(v**2)):.3f} max|{np.abs(v).max():.2f}|")
