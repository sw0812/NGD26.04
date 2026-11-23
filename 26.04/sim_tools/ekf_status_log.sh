#!/usr/bin/env bash
# 1초마다 EKF 융합 상태 + EV 혁신값 기록. 사용: ekf_status_log.sh out.txt seconds
B=~/바탕화면/KASA_SITL/PX4-Autopilot/build/px4_sitl_default/bin
for i in $(seq ${2:-400}); do
  f=$($B/px4-listener estimator_status_flags -n 1 | awk '/cs_ev_pos:|cs_gnss_pos:|cs_opt_flow:/{printf "%s%s ",$1,$2}')
  e=$($B/px4-listener estimator_aid_src_ev_pos -n 1 2>/dev/null | awk '/innovation_rejected:|fused:/{printf "%s%s ",$1,$2} /^ +innovation:/{printf "innov%s%s ",$2,$3}')
  p=$($B/px4-listener vehicle_local_position -n 1 | awk '/^ +(x|y|z):/{printf "%s%.2f ",$1,$2}')
  echo "$(date +%T) $p| $f| $e"; sleep 1
done
