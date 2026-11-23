# KASA 탐색 미션 SITL 실행 가이드 (머신 B, 2026-11-10)

머신 B = Ubuntu 26.04 x86_64, ROS2 Lyrical, gz-sim10, PX4 v1.18.0-beta1, QGC v5.1.5.
기체 = tarot680pro (airframe `6022_gz_tarot680pro`), 월드 = `kasa_indoor_search`.

## 1. 실행 순서 (터미널 5개)

```bash
# ① PX4 SITL + gz
cd ~/바탕화면/KASA_SITL && source /opt/ros/lyrical/setup.bash && source .venv-px4/bin/activate
cd PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridge
PX4_SIM_MODEL=gz_tarot680pro GZ_IP=127.0.0.1 PX4_GZ_WORLD=kasa_indoor_search \
PX4_GZ_MODEL_POSE="0,0,0.95,0,0,0" ~/바탕화면/KASA_SITL/PX4-Autopilot/build/px4_sitl_default/bin/px4

# ② ROS2 <-> PX4
MicroXRCEAgent udp4 -p 8888

# ③ QGC (조종기 매핑/스로틀 보정 포함 — 반드시 이 스크립트로)
~/QGroundControl/qgc.sh

# ④ gz 하향카메라 -> /camera/image_raw 브리지 + ArUco 검출
~/바탕화면/mission_logs_20261110/mission_stack.sh

# ⑤ 미션 노드
cd ~/바탕화면/KASA_SITL/01_ros2_ws_2604_MAIN/src/my_first_pkg
source /opt/ros/lyrical/setup.bash && source ~/ros2_ws/install/setup.bash
python3 search_phase_node.py --ros-args -p n_markers_expected:=4
```

선택: 영상처리/비전 화면 → `~/바탕화면/mission_logs_20261110/vision_view.sh`
(④가 떠 있어야 함. rqt 창 2개: `/line_tracker/debug`, `/aruco_tracker/debug`)

## 2. 미션 흐름 (search_phase_node.py)

이륙 2m → 격자 28칸 지그재그 탐색 (x 5~23, y 5~14, 3m 간격) → 마커 N개 다 찾으면 중단 →
ID 역순 구조 경로 → 버티포트(0,0) 복귀 → ID0 마커 보고 정밀착륙 → disarm → 노드 자동 종료.

| 기능 | 사용하는 것 |
|---|---|
| 이동 방향 | PX4 EKF 위치 (목표 칸 좌표로 속도 명령) |
| 교차점 도착 판정 | 라인트레이싱 (`line_tracker.py`), 못 보면 위치 0.15m 백스톱 |
| 마커 식별 | aruco_opencv (DICT_5X5_50, 0.5m) |
| 정밀착륙 | 버티포트 중앙 ArUco ID0 |

정답 마커 위치: ID1(8,5) ID2(11,14) ID3(17,8) ID4(23,14).

## 3. 결과 (2026-11-10)

| 회차 | 마커 | 착륙 |
|---|---|---|
| 1 | 4/4 | 착지 후 disarm 안 됨 → 수정 |
| 2 | 3/4 (ID2 누락) → 수정 | 정상 |
| 3 | **4/4** | **정상 (disarm + 노드 종료)** |

로그·ulog·상세: `~/바탕화면/mission_logs_20261110/README.md`

## 4. 오늘 수정한 것

- `search_phase_node.py` (백업 `.bak_1110`)
  - 착지 판정: 최종 하강 중 |vz|<0.03 m/s 1.5초 → disarm
  - ArUco 확인 프레임은 칸 0.6m 안에서만 차감
  - disarm 1초 뒤 노드 자동 종료
- `line_tracker_viz_node.py` 새로 작성 (라인트레이싱 디버그 영상 발행)
- `tarot680pro/model.sdf` (백업 `model.sdf.bak_cam`): 하향 카메라 z -0.25→-0.15, near clip 0.05→0.01
  (예전엔 카메라가 skid보다 아래라 착지하면 버티포트 상판 속에 파묻혔음). 미션 비행은 아직 이 값으로 안 돌려봄.
- 스폰 높이 `PX4_GZ_MODEL_POSE` z 0.75→0.9: 버티포트 높이 0.7 + skid 0.19라 0.75면 기체가 상판에
  박힌 채 시작해서 캠이 판 밑 바닥(회색 블록)을 찍었음. 확인 사진: `~/바탕화면/mission_logs_20261110/cam_check/`

## 5. 주의 / 미해결

- **미션 노드를 띄우기 전에 이전 노드가 꺼졌는지 확인** (`ps aux | grep search_phase_node`).
  두 개가 동시에 돌면 setpoint가 섞여 기체가 멈춤.
- QGC Joystick 설정 화면이 열려 있으면 조종 입력이 기체로 안 감 (Fly 화면으로).
- 하향 카메라 화각 115°(gz) vs 실기체 C270 47.5° — 라인이 2~3px로 가늘게 보임. 변경 여부 미결정.
- 스로틀 최상단 / 채널 반전 미확인.
- 규칙: GPS/EKF2 관련 파라미터·`COM_RC_IN_MODE`는 먼저 물어보고 바꿀 것.
  `TAROT680PRO_SITL.md` §1·§2 수치는 임의 변경 금지.
