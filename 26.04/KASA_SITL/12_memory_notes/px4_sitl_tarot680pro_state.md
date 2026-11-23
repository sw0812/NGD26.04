---
name: px4-sitl-tarot680pro-state
description: Current state and key findings for the tarot680pro PX4 SITL simulation/ArUco/mission project
metadata:
  node_type: memory
  type: project
  originSessionId: c262037b-e3a4-43c3-9299-846565ccd7e0
  modified: 2026-11-04T13:55:45.222Z
---

As of 2026-11-04, a PX4 SITL simulation is running in the background: gzserver (stock `empty.world`) + px4 (model `tarot680pro`) + gzclient, with params `EKF2_GPS_CTRL=3`, `EKF2_OF_CTRL=1`, `COM_RC_IN_MODE=3`. This combination arms successfully and was confirmed still running (PIDs 60313/60314/60318/60449 for sim, 76264 for gzclient) as of this session.

Key confirmed findings — do not re-investigate these, treat as settled:
- Fully disabling GPS (`EKF2_GPS_CTRL=0`) makes arming impossible in any flight mode. This is a structural circular-bootstrap problem in PX4 EKF2 source (`estimatorCheck.cpp`, `optical_flow_control.cpp`, `range_height_control.cpp`, `terrain_control.cpp`), not fixable via parameters — would require EKF2 code changes (out of scope).
- `EKF2_GPS_CTRL=3` on the stock world arms and holds normally — this is the working baseline.
- The custom KASA world (`kasa_gazebo_world`) still has an unresolved `ESTIMATOR_GPS_GLITCH` error. Ruled out: not a model issue, not a spawn-method issue (confirmed via stock-world A/B test). Root cause still unknown.

Infrastructure gotchas (apply immediately if they recur):
- `gzserver` must be launched with only `-s libgazebo_ros_init.so`. Adding `-s libgazebo_ros_factory.so` alongside it causes a hang. Need `/clock` and `/camera/image_raw` topics present.
- When running PX4 in the background, always use the `-d` (daemon) flag — without it the `pxh` shell spins infinitely and logs can balloon to tens of MB–GB.

Incomplete work:
- ArUco test marker (`kasa_gazebo_world/models/aruco_test_marker/`): never appears in the camera feed. Tried plane/box/RGB variants — all failed. World-embed approach also caused `gzserver` to hang. Currently on hold.
- `line_tracker.py` / `search_phase_node.py` (`~/ros2_ws/src/my_first_pkg/`): pre-existing, otherwise-complete KASA mission nodes. Only fix applied so far: updated `vehicle_status_v1` → `v4` topic version. Not yet run/tested.

See [[px4_sitl_user_preferences]] for how to handle GPS-param changes and debugging approach on this project.
