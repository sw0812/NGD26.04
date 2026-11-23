#!/usr/bin/env bash
source ~/바탕화면/sim_tools/kasa_ros_env.sh   # ROS 도메인 42 분리 (2026-11-12)
# KASA 탐색미션 밤샘 배치 (gz SITL, tarot680pro). Claude 없이 혼자 돔.
#   설정 A: 전방+하향 카메라, GPS 켬   B: 하향만, GPS 켬   C: 전방+하향, GPS 끔 + 라인EV + 안전장치
#   매 회차: 시뮬 재시작 -> 마커 4개 무작위 배치 -> 비행 -> 채점 -> results.csv / SUMMARY.md / summary.png 갱신
# 멈추기: touch ~/바탕화면/2026-11-11/0018_overnight_batch/STOP   (현재 회차 끝나면 종료)
# 사용: run_batch.sh [종료시각 HH:MM, 기본 08:00] [최대회수, 기본 200]
OUT=${OUT_DIR:-~/바탕화면/2026-11-11/0018_overnight_batch}; mkdir -p $OUT
SCRIPTS=~/바탕화면/sim_tools
# SEEDS="0 2 3" 이면 그 마커배치 seed만 순서대로 (같은 배치로 비교용). LO_ARGS: line_odometry 추가 ros-args
ML=~/바탕화면/sim_tools
B=~/바탕화면/KASA_SITL/PX4-Autopilot/build/px4_sitl_default
PKG=~/바탕화면/KASA_SITL/01_ros2_ws_2604_MAIN/src/my_first_pkg
PY=~/바탕화면/KASA_SITL/.venv-px4/bin/python
END=${1:-08:00}; MAXN=${2:-200}
END_TS=$(date -d "$END" +%s); [ $END_TS -lt $(date +%s) ] && END_TS=$(date -d "tomorrow $END" +%s)
source /opt/ros/lyrical/setup.bash; source ~/ros2_ws/install/setup.bash
CFG=(A B C C)
[ -n "$FORCE_CFG" ] && CFG=($FORCE_CFG)
kill_pat() { for p in $(ps -eo pid,args | awk -v pat="$1" '$0 ~ pat && !/awk/ {print $1}'); do kill $p 2>/dev/null; done; }
param() { $B/bin/px4-param set $1 $2 >/dev/null 2>&1; }
log() { echo "$(date '+%F %T') $*" | tee -a $OUT/batch.log; }

log "=== 배치 시작 (종료 $(date -d @$END_TS '+%F %T'), 최대 $MAXN회) ==="
i=$(ls -d $OUT/run_[0-9]* 2>/dev/null | wc -l)
SEEDS_ARR=($SEEDS); [ ${#SEEDS_ARR[@]} -gt 0 ] && MAXN=${#SEEDS_ARR[@]}
while [ $i -lt $MAXN ] && [ $(date +%s) -lt $END_TS ] && [ ! -f $OUT/STOP ]; do
  R=$(printf "%03d" $i); C=${CFG[$((i % ${#CFG[@]}))]}; RD=$OUT/run_$R; mkdir -p $RD
  MK=$(python3 -c "
import random; r=random.Random(${SEEDS_ARR[$i]:-$i})
cells=[(x,y) for x in (5,8,11,14,17,20,23) for y in (5,8,11,14)]
c=r.sample(cells,4); ids=[1,2,3,4]; r.shuffle(ids)
print(' '.join(f'{k}:{x}:{y}' for k,(x,y) in zip(ids,c)))")
  log "run_$R config=$C seed=${SEEDS_ARR[$i]:-$i} markers=[$MK]"
  kill_pat "python3 -u search_phase_node.py"; kill_pat "python3 -u line_odometry_node.py"; kill_pat "truth_guard.sh"
  for a in X Y Z; do param CAL_MAG0_${a}OFF 0; done   # 재시작 전 저장값도 초기화
  ok=0; for t in 1 2; do bash $ML/start_sim.sh > $RD/sim_start.log 2>&1 && { ok=1; break; }; log "  sim start retry"; done
  [ $ok = 1 ] || { log "  sim start FAILED -> skip"; i=$((i+1)); continue; }
  if [ $C = C ]; then param EKF2_EV_CTRL 1; param EKF2_GPS_CTRL 0; else param EKF2_GPS_CTRL 3; param EKF2_EV_CTRL 0; fi
  # 2026-11-11: 비행 중 잘못 학습된 지자기 오차가 CAL_MAG0_*OFF에 저장되면(05:33 이후 ZOFF 0.256) 이후 모든 시작이
  # "Strong magnetic interference"로 실패 -> 매 회차 0으로 초기화 (사용자 승인 2026-11-11)
  for a in X Y Z; do param CAL_MAG0_${a}OFF 0; done
  $ML/spawn_markers.sh "$MK" > $RD/spawn.log 2>&1; grep -q FAIL $RD/spawn.log && log "  marker spawn FAIL: $(cat $RD/spawn.log | tr '\n' ' ')"
  sleep 8
  cd $PKG
  FRONT=true; [ $C = B ] && FRONT=false
  EXTRA=""
  if [ $C = C ]; then
    setsid nohup python3 -u line_odometry_node.py --ros-args -p csv:=$RD/line_odom.csv $LO_ARGS > $RD/line_odom.log 2>&1 < /dev/null &
    setsid nohup $ML/truth_guard.sh > $RD/guard.log 2>&1 < /dev/null &
    EXTRA="-p require_line_odom:=true"; sleep 2
  fi
  timeout 600 python3 -u search_phase_node.py --ros-args -p n_markers_expected:=4 -p use_front_cam:=$FRONT $EXTRA > $RD/mission_node.log 2>&1
  rc=$?
  if [ $rc = 124 ]; then touch $RD/TIMEOUT; log "  TIMEOUT -> LAND"; $B/bin/px4-commander mode auto:land >/dev/null 2>&1; fi
  for t in $(seq 75); do $B/bin/px4-listener vehicle_status -n 1 2>/dev/null | grep -q " arming_state: 1" && break; sleep 2; done
  kill_pat "python3 -u line_odometry_node.py"; kill_pat "truth_guard.sh"
  param EKF2_GPS_CTRL 3; param EKF2_EV_CTRL 0
  sleep 3; U=$(ls -t $B/rootfs/log/*/*.ulg 2>/dev/null | head -1); [ -n "$U" ] && mv "$U" $RD/run.ulg
  $PY $SCRIPTS/eval_run.py $RD $C "$MK" $OUT/results.csv >> $OUT/batch.log 2>&1
  $PY $SCRIPTS/summarize.py $OUT >> $OUT/batch.log 2>&1
  log "  done: $(tail -1 $OUT/results.csv | cut -d, -f4-6,19,21,22)"
  i=$((i+1))
done
param EKF2_GPS_CTRL 3; param EKF2_EV_CTRL 0
log "=== 배치 종료 ($i회) ==="
