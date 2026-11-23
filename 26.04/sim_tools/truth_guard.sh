#!/usr/bin/env bash
# gz 실제 위치가 안전구역(x -4~32, y -4~23) 밖이면 즉시 AUTO.LAND + 미션 노드 종료. SITL 전용 안전장치.
export GZ_IP=127.0.0.1
B=~/바탕화면/KASA_SITL/PX4-Autopilot/build/px4_sitl_default/bin
while true; do
  p=$(timeout 3 gz model -m tarot680pro_0 -p 2>/dev/null | grep -A1 'Pose \[ XYZ' | tail -1 | tr -d '[]')
  x=$(echo $p | awk '{print $1}'); y=$(echo $p | awk '{print $2}')
  if [ -n "$x" ] && awk -v x=$x -v y=$y 'BEGIN{exit !(x<-4||x>32||y<-4||y>23)}'; then
    echo "$(date +%T) GUARD: truth ($x,$y) outside safe area -> LAND"
    for q in $(ps -eo pid,args | awk '/python3 -u search_phase_node.py/ && !/awk/ {print $1}'); do kill $q; done
    $B/px4-commander mode auto:land; exit 0
  fi
  sleep 1
done
