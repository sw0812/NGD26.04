# KASA → Raspberry Pi 5 + AI HAT(Hailo-10H) 이식

> 상태: **초안, 실기기 미검증.** 이 NUC에는 torch/ultralytics/Hailo 툴/Pi가 없어 변환·컴파일·추론은 실행해 보지 못함.
> 확인된 것: Python 문법, `kasa_hailo` 패키지 colcon 빌드(NUC, ROS Lyrical), 셸 스크립트 `bash -n`.

## 무엇이 NPU로 가고 무엇이 CPU에 남는가
| 구성 | 실행 위치 | 이유 |
|---|---|---|
| YOLOv8n landing_pad 추론 (`kasa_hailo/hailo_yolo_inference_node`) | **Hailo-10H NPU** | 신경망 |
| ArUco 검출 (`aruco_opencv`), 라인 검출/오도메트리, `search_phase_node` 미션 FSM, depth_map/landing 노드 | Pi CPU | 고전 CV·제어 로직. NPU 대상 아님 |

## 구성
- `model_convert/` (x86 PC): `export_onnx.py` → `make_calib.py` → `compile_hef.sh` → `landing_pad.hef`
- `kasa_hailo/` (ROS 2 ament_python 패키지): Hailo 추론 노드, `hailo_landing.launch.py`, 카메라/ArUco 설정
- `pi_deploy/`: `install_pi.sh`(워크스페이스+의존성), `run_stack_pi.sh [mission|landing|all]`

## 순서
1. x86 PC: `python3 model_convert/export_onnx.py ../KASA_SITL/08_datasets_models/best.pt 640`
2. x86 PC: `python3 model_convert/make_calib.py <lidar_dataset_yolo/images/train> calib_set.npy 640`
3. x86 PC(+Hailo DFC): `model_convert/compile_hef.sh best.onnx calib_set.npy` (`--hw-arch hailo10h`) → `landing_pad.hef` 을 Pi로
4. Pi: Hailo 런타임 설치 확인(`hailortcli fw-control identify`), `PX4_MSGS_BRANCH=<펌웨어 맞는 브랜치> pi_deploy/install_pi.sh`
5. Pi: `HEF=~/landing_pad.hef pi_deploy/run_stack_pi.sh all`

## 먼저 확인할 위험 요소
1. **OS/ROS**: 스크립트는 Ubuntu 24.04 + Jazzy 가정. Raspberry Pi OS는 ROS 바이너리가 없고, Ubuntu는 Hailo 패키지를 수동 설치해야 함(`hailo-h10-all` 등 패키지명은 Pi 문서로 확인).
2. **컴파일**: `compile_hef.sh` 의 끝단 노드·NMS `convNN` 이름은 모델별로 다름 — parser 로그/netron 으로 맞출 것. 학습 imgsz 와 export imgsz 불일치 주의(`best.pt` 의 imgsz 는 torch 없이 못 읽음, 640 가정).
3. **정확도**: HEF 결과를 같은 이미지로 ONNX/PT 와 비교(INT8 양자화 손실). 패드 프레임이 적으면 캘리브레이션 품질 하락.
4. **NMS 출력**: `parse_nms()` 는 HailoRT 버전별 형식 가정 — 첫 실행 로그의 output shape 보고 수정.
5. **하드웨어**: 기존 카메라 설정은 Jetson+C270 기준(`c270_usb_cam.yaml`). Pi에서 `/dev/video0` 번호/MJPG·YUYV 동작 확인. 전방 카메라(`use_front_cam`)는 두 번째 카메라가 있을 때만.
6. **px4_msgs**: 기체 PX4 펌웨어 버전과 브랜치가 맞아야 함. XRCE-DDS 에이전트와 `ROS_DOMAIN_ID` 도 맞출 것.
7. **성능**: Pi CPU 에서 ArUco+라인오도메트리+미션 노드가 시뮬 FPS(≈카메라 30Hz)를 유지하는지 측정 필요.
