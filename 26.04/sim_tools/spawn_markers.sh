#!/usr/bin/env bash
# 사용: spawn_markers.sh "1:8:5 2:11:14 3:17:8 4:23:14"   (ID:x:y, world ENU, 교차점)
export GZ_IP=127.0.0.1
Z=${MARKER_Z:-0.01}
for spec in $1; do
  IFS=: read id x y <<< "$spec"
  timeout 10 gz service -s /world/kasa_indoor_search/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
    --req "sdf: '<?xml version=\"1.0\"?><sdf version=\"1.9\"><include><uri>model://kasa_marker_${id}</uri><name>kasa_marker_${id}</name></include></sdf>' pose: {position: {x: $x, y: $y, z: $Z}}" >/dev/null 2>&1 \
    && echo "spawned ID$id @ ($x,$y)" || echo "FAIL ID$id"
done
