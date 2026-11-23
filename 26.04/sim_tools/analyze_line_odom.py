"""line_odom CSV: 오차를 속도(지연) / 기울기(자세보정) 로 회귀해서 원인 분리."""
import sys, numpy as np
d = np.genfromtxt(sys.argv[1], delimiter=',', names=True)
for ax, v, tilt in [('err_n', 'vn', 'pitch'), ('err_e', 've', 'roll')]:
    m = ~np.isnan(d[ax]) & (d['h'] > 1.5)
    e, vel = d[ax][m], d[v][m]
    # 기울기: NED 기준 북쪽 이동은 pitch(-), 동쪽 이동은 roll(+) 이지만 yaw에 따라 섞이므로 둘 다 넣음
    A = np.c_[vel, d['pitch'][m], d['roll'][m], np.ones(m.sum())]
    coef, *_ = np.linalg.lstsq(A, e, rcond=None)
    res = e - A @ coef
    print(f"{ax}: n={m.sum()} rms={np.sqrt(np.mean(e**2)):.3f}  fit: {coef[0]:+.3f}*v {coef[1]:+.3f}*pitch {coef[2]:+.3f}*roll {coef[3]:+.3f}  resid rms={np.sqrt(np.mean(res**2)):.3f}")
    slow = m & (np.abs(d[v]) < 0.3)
    print(f"   |v|<0.3: n={slow.sum()} mean={np.nanmean(d[ax][slow]):+.3f} std={np.nanstd(d[ax][slow]):.3f}")
    print(f"   corr(err,v)={np.corrcoef(e, vel)[0,1]:+.2f}")
