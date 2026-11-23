# KASA 실내조난자 탐색 드론 — gz SITL 인수인계 (2026-11-11 기준)

> **새 Claude 대화에서 이 파일부터 읽힐 것.** "~/바탕화면/read.md 읽고 이어서 해" 하면 바로 이어갈 수 있게 씀.
> 자세한 결과 정리: `~/바탕화면/20261111_시뮬정리.md` / 비행 그래프: `~/바탕화면/20261111_비행로그/`

---

## 0. 사용자 규칙 (반드시 지킬 것)

- 한국어, 짧고 직접적으로. 사용자는 말이 거칠지만 "니가 해" = 직접 실행해 달라는 뜻.
- **GPS/EKF2/센서 파라미터(`EKF2_GPS_CTRL`, `EKF2_EV_CTRL`, `EKF2_OF_*`, `COM_RC_IN_MODE`, `CAL_MAG*` 등)는 바꾸기 전에 먼저 물어볼 것.**
  예외(이미 승인됨): 배치 스크립트가 C 설정 회차에만 `EKF2_GPS_CTRL 0 / EKF2_EV_CTRL 1`로 바꾸고 끝나면 `3 / 0`으로 복구,
  매 회차 `CAL_MAG0_X/Y/ZOFF = 0` 초기화.
- `KASA_SITL/03_px4_customizations/TAROT680PRO_SITL.md` §1(물리)·§2(로터 배치) 수치는 임의 변경 금지, 바꿔야 하면 먼저 알릴 것.
- **크레딧 민감** — 추측성 시행착오 반복 금지. 한 번에 하나씩, 원인 확인 후 고칠 것. 긴 시험은 배치 스크립트로 맡기고 기다릴 것.
- 결과는 과장 없이. "성공"이라고 하기 전에 실제 위치(gz groundtruth)로 채점한 근거를 댈 것.
- 프로젝트 메모: `~/바탕화면/KASA_SITL/12_memory_notes/` (px4_sitl_tarot680pro_state.md, px4_sitl_user_preferences.md)

---

## 1. 환경 (머신 B)

| 항목 | 값 |
|---|---|
| OS | Ubuntu 26.04 x86_64, Wayland (QGC는 XWayland/xcb) |
| ROS2 | Lyrical (`source /opt/ros/lyrical/setup.bash`), 추가 ws `~/ros2_ws` (px4_msgs, aruco_opencv) |
| 시뮬 | gz-sim10 (`/opt/ros/lyrical/opt/gz_sim_vendor`), PX4 v1.18.0-beta1 (`~/바탕화면/KASA_SITL/PX4-Autopilot`) |
| 기체 | airframe `6022_gz_tarot680pro`, 모델 `Tools/simulation/gz/models/tarot680pro/model.sdf` |
| 월드 | `kasa_indoor_search` (버티포트 = 원점 (0,0), 상판 높이 0.7 m) |
| 파이썬(ulog 분석) | `~/바탕화면/KASA_SITL/.venv-px4/bin/python` (pyulog, matplotlib 있음) |
| QGC | `~/QGroundControl/qgc.sh`로만 실행 (Boxer 조종기 매핑+스로틀 보정 포함) |

좌표: world ENU (x=East, y=North). PX4 local NED의 (x,y) = world (y,x). 미션 격자 교차점 x∈{5,8,…,23}, y∈{5,8,11,14} (28개),
격자선 x∈{2,5,…,26}, y∈{2,5,…,17}, 안전구역 테두리 x=-2/30, y=-2/21.

---

## 2. 실행 순서 (처음부터 손으로 띄울 때)

```bash
D=~/바탕화면/mission_logs_20261110
# ① PX4 SITL + gz (기존 인스턴스 정리 후 데몬으로 기동, READY 대기)
bash $D/start_sim.sh            # 스폰 (0,0,0.95). "READY" 나와야 함
# ② ROS2 <-> PX4
setsid nohup MicroXRCEAgent udp4 -p 8888 > $D/xrce.log 2>&1 < /dev/null &
# ③ 카메라 브리지(하향+전방) + ArUco 2개(하향 /aruco_detections, 전방 /aruco_front/detections)
setsid nohup $D/mission_stack.sh > $D/mission_stack.log 2>&1 < /dev/null &
# ④ (선택) 화면: 라인트레이싱/ArUco/전방 카메라 rqt 창
$D/vision_view.sh &
# ⑤ 마커 무작위 배치 (ID:x:y, 교차점)
$D/spawn_markers.sh "1:8:5 2:11:14 3:17:8 4:23:14"
# ⑥ 미션 노드
cd ~/바탕화면/KASA_SITL/01_ros2_ws_2604_MAIN/src/my_first_pkg
source /opt/ros/lyrical/setup.bash && source ~/ros2_ws/install/setup.bash
python3 -u search_phase_node.py --ros-args -p n_markers_expected:=4            # GPS 켬 (A)
#   하향만(B):  -p use_front_cam:=false
#   GPS 끔(C):  (PX4: param set EKF2_EV_CTRL 1; param set EKF2_GPS_CTRL 0) + 아래 라인위치 노드 + -p require_line_odom:=true
python3 -u line_odometry_node.py --ros-args -p csv:=/tmp/x.csv                 # C일 때만
```

**보통은 배치 스크립트로 돌림 (3절).** 손으로 할 땐 이전 미션 노드가 꺼졌는지 반드시 확인 (두 개 돌면 setpoint 섞여 기체가 멈춤).
PX4 명령은 `~/바탕화면/KASA_SITL/PX4-Autopilot/build/px4_sitl_default/bin/px4-{param,listener,commander}`.

---

## 3. 배치 시험 (주력 도구)

`~/바탕화면/overnight_batch/run_batch.sh [종료시각 HH:MM] [최대회수]` — Claude 없이 혼자 돔.
매 회차: 지자기 보정 초기화 → 시뮬 재시작 → 파라미터 설정 → 마커 4개 무작위 배치(seed=회차번호) → 미션(10분 제한) → 착륙 대기 →
파라미터 복구 → ulog 저장 → 채점(`eval_run.py`) → `results.csv`, `SUMMARY.md`, `summary.png` 갱신.

| 환경변수 | 뜻 |
|---|---|
| `FORCE_CFG="C"` | 설정 고정 (기본 순환 A B C C) |
| `SEEDS="0 2 3"` | 그 마커배치만 (같은 배치 1:1 비교용) |
| `OUT_DIR=~/바탕화면/xxx` | 결과 폴더 (기본 overnight_batch) |
| `LO_ARGS="-p meas_std:=0.1 -p max_innov:=0.5"` | 라인위치 노드 인자 (C) |

설정: **A** 전방+하향, GPS 켬 / **B** 하향만, GPS 켬 / **C** 전방+하향, GPS 끔 + 라인위치 EV + 안전장치 2겹.
예: `OUT_DIR=~/바탕화면/test1 FORCE_CFG=C SEEDS="0 2 3" setsid nohup ~/바탕화면/overnight_batch/run_batch.sh 23:59 3 > /dev/null 2>&1 &`
멈추기: `touch <OUT_DIR>/STOP` (현재 회차 끝나고 종료). 진행: `tail <OUT_DIR>/batch.log`.

분석 도구 (전부 `.venv-px4/bin/python`으로):
- `mission_logs_20261110/plot_flight.py <ulg> <png> "제목" "ID:x:y ..."` — 궤적/오차/고도/융합 상태 그림
- `mission_logs_20261110/score_arrivals.py <mission.log> <ulg>` — 교차점 도착 판정 채점 (시간축 자동 정렬)
- `mission_logs_20261110/compare_runs.py A.ulg 이름 B.ulg 이름 out.png` — 두 비행 비교
- `overnight_batch/ensemble_top40.py`, `plot_top40.py` — 시간평균/앙상블 상위 40% 필터
- `ensemble_param_C/compare_paired.py` — 같은 배치 1:1 비교

---

## 4. 현재 결과 (밤샘 84회, 마커 무작위)

| 설정 | 4/4 정답 | 마커 정답률 | 틀린 칸 | 시간평균 EKF오차 | 최대오차 평균 | 이탈/안전장치 |
|---|---|---|---|---|---|---|
| A 전방+하향, GPS 켬 | 21/21 | 100% | 0 | ~0.155 m | 0.80 m | 0/0 |
| B 하향만, GPS 켬 | 10/21 | 82% | 0 | ~0.14 m | 0.74 m | 0/0 |
| C GPS 끔 + 라인위치 | 42/42 | 100% | 0 | **~0.09 m** | **0.36 m** | 0/0 |

- 전방 카메라 = 마커 찾기 핵심 (B만 자주 놓침). C(GPS 끔)가 위치 정확도 최고 (단, 시뮬 격자선이 완벽해서 유리).
- 시간평균·앙상블 상위 40% 필터 결과 31/34 일치 (에르고딕).
- 앙상블 통계로 라인위치 버림기준 0.29 m 시험 → **악화**(측정 5~6배 더 버림). 기본값 0.10 / 0.5 유지.
- 주의: A·B·C가 서로 다른 마커배치로 날았음 (평균 비교만 유효).

---

## 5. 코드 구조 (`~/바탕화면/KASA_SITL/01_ros2_ws_2604_MAIN/src/my_first_pkg/`, colcon 안 씀, 직접 python3 실행)

| 파일 | 역할 |
|---|---|
| `search_phase_node.py` | 미션 상태머신: 이륙 2 m → 28칸 지그재그(SEARCH) → 마커 N개 모이면 ID 역순 구조경로(VISIT_ORDERED) → 버티포트 복귀 → ID0 정밀착륙 → disarm 1초 뒤 자동종료 |
| `grid_line_follow.py` | 격자선 검출 (blackhat → 덩어리 제거 → 다리 마스킹 → Hough). 임계값은 blackhat을 40으로 잘라 Otsu |
| `line_guidance.py` | 검출선을 자세·라이다로 지면 투영 → 진행선 횡오프셋 lat, 교차선 전방거리 crosses (m) |
| `line_odometry_node.py` | 격자선 → 절대위치(가장 가까운 실제 격자선에 스냅) → `/fmu/in/vehicle_visual_odometry` (C에서 EKF2 EV로 융합) |
| `line_tracker.py` | 예전 판정기 (legacy, `-p intersection_detector:=legacy`) |
| `line_tracker_viz_node.py` | `/line_tracker/debug` 화면 |

`search_phase_node.py` 주요 동작/파라미터:
- 비행 유도는 EKF 위치로 목표 칸까지 속도 명령 (FORWARD_SPEED 2.0, 칸 가까우면 감속). 고도 유지 2.0 m (ALT_HOLD_KP 0.8).
- 진행 방향으로 기수 정렬 (직진 중에도 yaw 유지) — 하향 카메라에 격자선이 세로로 서게.
- 교차점 도착: 격자선 판정(교차선 |fwd|<0.15 m & 진행선 |lat|<0.4 m, **최근 5프레임 중앙값**) 또는 EKF 대비책(목표 0.15 m 안).
  도착 때마다 `[ARRIVE] src=grid|ekf|legacy ... px4_t=...` 로그 (채점용).
- 마커 확인: 도착 후 하향 ArUco 투표(15프레임, 0.6 m 안, 화면중심 0.47 m 안), 4초 타임아웃.
- 전방 카메라 선등록 (`use_front_cam`, 기본 True): 6 m 안 마커 → world 좌표 → 가장 가까운 교차점 투표 8표·75% → 등록. 하향이 다른 칸에서 보면 하향으로 수정.
- 안전장치 (`require_line_odom:=true`, C): 미션구역(x 3~25, y 3~16) 안에서 라인위치 3초 끊김 → 제자리 정지/복귀 시 재개, 10초 → AUTO.LAND.
- 착지 판정: 고도 0.05 m 이하 또는 최종하강 중 |vz|<0.03 m/s 1.5초.
- 백업: 수정마다 `search_phase_node.py.bak_*` (최근: `.bak_wd2`, `.bak_alt`, `.bak_front`, `.bak_median`).

`line_odometry_node.py`: `meas_std`(기본 0.10 m), `max_innov`(0.5 m, 예측과 이보다 다르면 버림), 버티포트 2.5 m 안 미측정,
상판 테두리/V 선 버림, position 3축 모두 숫자여야 PX4가 받음(z는 EKF값). 2초마다 `[line-EKF]`, `[why]`(버린 이유 카운트) 로그.

---

## 6. 시뮬 모델/월드 변경 사항 (원본 백업 있음)

| 대상 | 변경 | 백업 |
|---|---|---|
| tarot680pro `model.sdf` | KM 부호 문서 규약(0:-,1:+,2:-,3:+,4:+,5:-) — airframe 6022 | `6022...bak_km` |
| 〃 | 하향카메라 z -0.15, near clip 0.01 | `model.sdf.bak_cam` |
| 〃 | 전방카메라 35° 하향 (pitch 0.6109) | `model.sdf.bak_front` |
| 〃 | skid 충돌을 다리 메쉬에 맞춤 (z -0.233, y ±0.17, 0.38 m) | `model.sdf.bak_skid` |
| kasa_ground_field 텍스처 | 구운 마커 4개 지운 버전 사용 (+블록 무늬) | `field_texture_with_markers.png`, `.orig_bak` |
| 마커 모델 | `models/kasa_marker_1~4` (DICT_5X5_50, 0.5 m, 텍스처 `kasa_aruco_5x5_idN.png`, 바닥 위 1 cm) | — |
| 스폰 높이 | 0.95 m (`start_sim.sh`) | — |

모델 사본은 `KASA_SITL/03_px4_customizations/models_gz_sim/tarot680pro/`에도 복사해 둠.

---

## 7. 함정 (이미 겪은 것 — 다시 디버깅하지 말 것)

1. **`pkill -f 패턴` / `pgrep -f`가 자기 셸까지 죽임** (명령줄에 같은 문자열) → `pgrep -x <comm>` 또는 `ps -eo pid,args | awk '/패턴/ && !/awk/'`.
2. **gz 텍스처 캐시가 파일 이름 기준** — 모델마다 텍스처 파일명 달라야 함 (다 `marker.png`면 전부 첫 마커로 보임).
3. **지자기 학습값 저장**: GPS 끈 비행 후 `CAL_MAG0_ZOFF`가 0.256처럼 저장되면 이후 모든 시작이 "Strong magnetic interference"로 실패 → 0으로 (배치는 자동).
4. **SITL `param set`은 rootfs/parameters.bson에 저장돼 재시작해도 남음** — 바꾼 건 꼭 복구.
5. ROS Lyrical rclpy엔 `logger.warn` 없음 → `logger.warning`.
6. ROS 쪽 PX4 메시지 `timestamp`는 XRCE timesync로 **유닉스 시간축**, ulog는 부팅 기준 → 채점 시 EKF 위치로 오프셋 자동 정렬(score_arrivals.py 방식).
7. 시뮬 막 켠 직후 하향캠 회색 블록 = 스폰이 낮아 상판에 박힌 것 (스폰 0.95로 해결).
8. 마커 칸에서 격자선 0개 = Otsu가 마커 기준으로 잡힘 (blackhat 40 클립으로 해결).
9. 버티포트 위에선 라이다가 상판(0.7 m)까지 재서 바닥 선 투영 틀림 → 2.5 m 안 라인위치 안 씀.
10. 격자는 3 m마다 똑같음 → 라인위치는 "EKF가 1.5 m 안으로 맞을 때 다듬는 장치". EKF가 크게 틀어지면 옆 선에 잠김(8회차 545 m 이탈 사고).
11. 필드 밖 무늬 없는 바닥에선 옵티컬플로우가 "정지"로 보고 → 미션구역 밖 비행 위험.
12. QGC Joystick 설정 화면 열려 있으면 조종 입력 안 감 (Fly 화면으로).

---

## 8. 다음 할 일 (우선순위)

1. **같은 마커배치로 A/B/C 1:1 비교** (예: `SEEDS` 20개 × 3설정 ≈ 4시간) — 지금은 설정마다 배치가 달라 B 마커 누락 원인 단정 불가.
2. **B가 마커 놓치는 원인**: 하향만일 때 마커 칸에서 확인창(15프레임/4초)이 비어서 닫힘 — `run_005`(2/4) 로그로 시작.
3. 라인위치 버림기준을 **측정-예측 차이** 분포(`line_odom.csv`의 err_n/err_e)의 99%로 정해 C 재시험 (EKF 오차 기준은 실패했음).
4. 실기체 근접: 하향카메라 화각 115° → C270 47.5°(fx=726.6@640) 로 바꾼 C 시험 (모델 변경, 사용자 미결정).
5. 회전 교차점에서 EKF 오차 튐(A·B 0.6~1.2 m) 완화.
6. (보류) 라인트레이싱으로 목표점 보정 — EKF 기준 목표가 실제 교차점보다 ~0.27 m 앞이라 카메라 도착판정 기회가 적음.

---

## 9. 파일 지도

| 경로 | 내용 |
|---|---|
| `~/바탕화면/read.md` | 이 문서 |
| `~/바탕화면/20261111_시뮬정리.md` | 결과 상세 정리 |
| `~/바탕화면/20261111_비행로그/` | 설정별 대표/최악/최고 비행 그림 + 미션로그 + 최고 회차 .ulg |
| `~/바탕화면/overnight_batch/` | 배치 스크립트·결과 84회 (`run_NNN/`에 mission_node.log, run.ulg, line_odom.csv …) |
| `~/바탕화면/ensemble_param_C/` | 앙상블 파라미터 C 시험 10회 |
| `~/바탕화면/mission_logs_20261110/` | start_sim/mission_stack/vision_view/spawn_markers 스크립트, 1~17회차 로그·분석 |
| `~/바탕화면/run10_log_GPS켬/` | 10회차 로그 |
| `~/바탕화면/read_line_odometry_20261110.md` | 라인위치 추정 상세(이전 read.md) |
| `~/바탕화면/KASA_SITL/read.md`, `take.md`, `README.md` | 프로젝트 전체 문서 (Classic vs gz, 조이스틱 등) |
| `~/바탕화면/KASA_SITL/12_memory_notes/` | 프로젝트 규칙·확정 사항 |

현재 상태: 시뮬·노드 모두 꺼져 있음 (새로 시작하려면 2절 ①~③).

---

## 10. 2026-11-11 저녁 추가 (재부팅 전 상태) — 새 세션은 여기부터

**카메라 = 라즈베리파이 글로벌셔터(IMX296, 1456x1088, 3.45um, 60 Hz)로 교체.** fx = 초점거리[mm]/0.00345.
- 현재 model.sdf: **하향 2.8 mm (HFOV 1.4623 rad, fx 811.6) / 전방 44.6° (0.7784 rad, fx 1775.0)**. 백업: `.bak_cam115`(원래 115°), `.bak_gs16`, `.bak_gs6`, `.bak_gs16b`, `.bak_fov446`
- 16 mm: 격자선 0회·마커 2/4·착륙 불가(0.5 m ID0가 화면보다 큼) -> 탈락. 6 mm/44.6°: 4/4 착륙 OK.
- fx 쓰는 곳: `line_guidance.py`(env `DOWN_FX`, 기본 811.6), `line_odometry_node.py` fx 811.6, `fused_line_node.py` `fx`(하향)/`fx_front`.
- 착륙 ①②(높게 접근/마커 없이 하강) 시도 후 **사용자가 취소** -> `search_phase_node.py`는 `.bak_land16` 상태 (취소본 `.land12_cancelled`).
- 순항 "2 m"는 home(버티포트 상판) 기준 -> 바닥 위 실제 약 2.98 m.

**새 측정 노드 (측정 전용, 제어 안 씀):** `fused_line.py`, `fused_line_node.py` — Canny+HoughP, 윤곽 극성 쌍(폭 0.03~0.25 m)으로 선 판정(픽셀 굵기 기준 X, 사용자 지시), 전방+하향 지면투영 융합. 채점 `mission_logs_20261110/score_fused.py fused.csv run.ulg`.
- 기존 `grid_line_follow.py`(blackhat 15px)는 새 카메라에서 격자선 0회 — 손 안 댐.

**렌즈 비교 배치 (진행 중 정지):** `~/바탕화면/lens_batch/` — 하향x전방 {2.1,2.8,4,6 mm} 16조합 x seed {1000,1001,1002}, 설정 A, 회당 ~3.2분.
- 결과 `SUMMARY.md`, `combos.csv` (`lens_summary.py`가 갱신). run_000~ 순서: seed 바깥, 하향, 전방 순.
- **재개:** 먼저 `cp lens_batch.sh.resume_fixed lens_batch.sh` (재시작 시 끝난 조합 건너뛰는 수정본), `rm STOP`,
  XRCE Agent + `mission_stack.sh` 먼저 띄운 뒤(2절 ②③ — 배치는 이 둘을 안 띄움!),
  `setsid nohup ~/바탕화면/lens_batch/lens_batch.sh > ~/바탕화면/lens_batch/stdout.log 2>&1 < /dev/null &`
- 배치 끝나면 model.sdf 자동 원복(하향 2.8/전방 44.6°).

**규칙 추가:** 시뮬/배치는 사용자가 명시적으로 시킬 때만 실행 (read.md "다음 할 일"은 허락 아님).

**2026-11-12 01:24 렌즈 배치 완료 (48회) -> 하향 4 mm / 전방 4 mm 적용** (HFOV 1.1206, fx 1159.4, 둘 다). 결과 `~/바탕화면/lens_batch/SUMMARY.md`.
- 상위 10조합 모두 마커 12/12·착륙 3/3 — 차이는 회차 간 편차 수준. 하향 6 mm는 lat95 0.29~0.47 m·psi95 15~20°로 확실히 나쁨, 전방 6 mm는 모서리 칸(23,8) 마커 2회 놓침.
- 이전(하향 2.8/전방 44.6°) 백업: `.bak_d28_f446`. start_sim.sh는 시작마다 CAL_MAG0_*OFF=0 (사용자 지시).

## 11. 2026-11-12 상태 + 앞으로 해결할 문제
- 현재 model.sdf: **전방+하향 3.2 mm** (HFOV 1.3309, fx 927.5). 4 mm 백업 `.bak_44_before32`. 노드 실행 시 `DOWN_FX=927.5`, line_odometry `-p fx:=927.5` 필수(기본값은 아직 1159.4).
- GPS끔 40회 스윕(4 mm, `gpsoff_batch/`): 39/40, 최적 EVP_NOISE 0.10 / EV_DELAY 40 ms (적용은 승인 대기, 현재 0 ms).
- 3.2 mm GPS끔 10회(`lens32_batch/`): 10/10, EKF오차 0.076 m. 하향 측정오차 ~1 cm로 4 mm와 같음, 전방 ArUco 스냅오차 0.12(최대 0.61) m로 4 mm보다 나쁨.
- 로그 정리: `~/바탕화면/1111log/`

해결할 문제 (우선순위):
1. 복귀 대각선 구간(격자선 없음) EKF 오차 0.12~0.2 m -> 복귀를 격자선 따라 꺾기 / ID0 조기 보정 / 등록 마커 재방문 보정
2. 미션노드 위치 처리 지연 최대 1.3 s (1.58 MP 60 Hz 다중 노드 부하) -> 도착판정·채점 왜곡
3. run_037: 착지 후 disarm 안 됨 + 지상 EKF 고도 표류(기압계, GPS끔)
4. 미션노드 도착판정이 옛 grid_line_follow 사용 (새 카메라에서 격자 도착 0회, EKF 도착만)
5. EKF2_EV_DELAY 40 ms 적용 여부 (사용자 승인)
6. 렌즈 확정(3.2 vs 4 mm, 전방 4 mm 구하면 하향3.2/전방4) 후 fx 기본값 갱신
7. 실기체 대비: 렌즈 왜곡 캘리브레이션(시뮬엔 왜곡 없음), 카메라 질량 0.03 kg -> 실물(승인 필요), front cam offset 코드 0.02 vs 모델 불일치(4 cm)

### 11-2. 갱신 (2026-11-12 19:40)
- 1번(복귀 대각선) **해결**: `search_phase_node.py` grid_return (마지막 마커 -> (5,5) -> (2,5) -> (2,2) -> 버티포트). 백업 `.bak_gridreturn`.
- 라인값 30회(`lineparam_batch/SUMMARY.md`): meas_std 0.07 6/6 최적, 0.15 이상 + max_innov<=0.5 -> 한 칸 어긋남 실패 7회. max_innov>0.7은 SNAP_TOL 0.7과 같음.
- truth_guard 경계는 x -4~32 / y -4~23 (필드 테두리 -2/30/21보다 2 m 바깥) -> 이탈 1.16 m까진 안 걸림(설계대로).

남은 문제 (우선순위):
1. **한 칸 어긋남 감지/복구** — EKF가 3 m 틀린 위치에 고정되면 못 돌아옴. 등록 마커·버티포트 ID0·테두리선(간격 4 m로 다름)으로 감지해서 바로잡기
2. 착륙 때 ID0 못 찾으면 무한 대기 (run_003 200 s 맴돎) -> 시간 제한 후 그 자리 AUTO.LAND
3. meas_std 기본값 0.10 -> 0.07 (사용자 결정) + 무작위 마커로 재확인
4. 미션노드 위치 처리 지연 최대 1.3 s
5. run_037 착지 후 disarm 안 됨 + 지상 EKF 고도 표류
6. 미션노드 도착판정 grid 검출 거의 0회 (lineparam 평균 0.17회/비행) -> EKF 도착만 씀
7. EKF2_EV_DELAY 40 ms 적용 여부 (사용자 결정)
8. 렌즈 확정 후 fx 기본값 갱신 (지금 코드 기본 1159.4 = 4 mm, 모델은 3.2 mm)
9. 실기체: 렌즈 왜곡 캘리브레이션, 카메라 질량(승인 필요), 전방카메라 오프셋 코드/모델 4 cm 불일치

### 11-3. 2026-11-12 23:50 — 네트워크 오염 사고 + 해결
- 19:30 이후 비행 전부 실패 원인: **연구실 다른 사람 PX4 SITL이 같은 와이파이, ROS 도메인 0** -> 위치 토픽 발행자 2개, 우리 PX4에 남의 setpoint가 섞임.
- 해결: `mission_logs_20261110/kasa_ros_env.sh` (ROS_DOMAIN_ID=42, DISCOVERY_RANGE=LOCALHOST), 주요 스크립트 2번째 줄에서 source. PX4가 UXRCE_DDS_DOM_ID=42 자동 설정.
  손으로 ros2 명령 쓸 땐 터미널에서 먼저 `source ~/바탕화면/mission_logs_20261110/kasa_ros_env.sh` (.bashrc 미반영).
- 무효 결과: lostland_test, reloc_test1(.bad_run1~3), ab_noreloc, diag_test/old_contaminated. 유효: lineparam_batch까지.
- 분리 후: T1 GPS켬 4/4 (0.173 m), T2 화면끔+GPS끔 4/4 (0.070/0.29 m, 프레임 58/2s, RELOC 오리셋 0, ArUco-EKF 차이 최대 0.22 m).
- 새 코드(검증 대기): RELOC(한 칸 어긋남 복구, line_odometry reloc 파라미터), 착륙 ID0 미검출 5 s -> AUTO.LAND (search_phase LANDING_LOST_*).
- 2026-11-13 00:14 RELOC 2단계: `reloc_test2/SUMMARY.md` — 리셋 3/3 정확(오차 ~1 m -> 0.0~0.08 m), 오리셋 0, 위험 설정에서 3/5 성공(RELOC 없을 때 0/3). 버티포트 근처는 1초 연속 발행(RELOC_STREAM)으로 수정, 아직 실비행 확인 안 됨.
- 2026-11-13: line_odometry MEAS_STD 기본값 0.10 -> **0.08** (사용자 결정). 0.08은 30회 스윕에 없던 값(0.07/0.10 사이) — 실비행 확인 전.
