# KASA 탐색 미션 SITL 로그 (2026-11-10, 머신 B / gz-sim / tarot680pro)

## 실행 순서
1. PX4 SITL + gz (kasa_indoor_search), MicroXRCEAgent udp4 -p 8888
2. `./mission_stack.sh` — gz 하향카메라 → /camera/image_raw, /camera/camera_info 브리지 + aruco_tracker(5X5_50, 0.5m)
3. `cd 01_ros2_ws_2604_MAIN/src/my_first_pkg && python3 search_phase_node.py --ros-args -p n_markers_expected:=4`

## 결과
| 회차 | 마커 | 착륙 | 비고 |
|---|---|---|---|
| 1 (run1) | 4/4 정답 위치 (ID1 8,5 / ID2 11,14 / ID3 17,8 / ID4 23,14) | 버티포트 착지했으나 disarm 안 됨 | EKF 고도 0.16m에 멈춰 0.05m disarm 조건 미충족 → 수동 disarm |
| 2 (run2) | 3/4 (ID2 누락) | 정렬·착지·disarm 정상 | 착지판정 수정 적용 |
| 3 (run3) | **4/4 정답 위치** | 정렬·착지·disarm·노드 자동종료 정상 | 모든 수정 적용. 교차점 도달 30회 중 비전 판정 26 / 위치 백스톱 4 |

## 노드 수정 (search_phase_node.py, 백업 .bak_1110)
- 착지 판정: 최종 하강 중 |vz|<0.03 m/s 1.5초 유지 시 disarm (run1 문제)
- ArUco 확인 프레임 카운트: 칸 0.6m 이내일 때만 차감 (run2 ID2 누락 — 1.5m 앞 비전 교차점 판정으로 접근 중 15프레임 소진)
  → run3에서 ID2 정상 검출로 확인
- disarm 1초 뒤 노드 자동 종료 (run3 첫 시도: run2 노드가 안 꺼져 setpoint 충돌 → 기체가 두 목표 사이에서 정지)

## 영상처리/비전 화면
- `vision_view.sh`: line_tracker_viz_node.py(/line_tracker/debug) + rqt_image_view 2개(라인트레이싱, /aruco_tracker/debug)

## 파일
- run*_mission_node.log: 미션 노드 출력
- run1_track_2s.log: 2초 간격 PX4 local position(NED) / nav_state
- *.ulg: PX4 비행 로그 (run3.ulg = 3회차)
