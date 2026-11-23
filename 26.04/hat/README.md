# KASA → Raspberry Pi 5 + AI HAT(Hailo-10H) 이식

> 상태: **초안, 실기기 미검증.** 이 NUC에는 torch/ultralytics/Hailo 툴/Pi가 없어 변환·컴파일·추론은 실행해 보지 못함.
> 확인된 것: Python 문법, `kasa_hailo` 패키지 colcon 빌드(NUC, ROS Lyrical), 셸 스크립트 `bash -n`.

## 무엇이 NPU로 가고 무엇이 CPU에 남는가
| 구성 | 실행 위치 | 이유 |
|---|---|---|
| YOLOv8n landing_pad 추론 (`kasa_hailo/hailo_yolo_inference_node`) | **Hailo-10H NPU** | 신경망 |
| ArUco 검출 (`aruco_opencv`), 라인 검출/오도메트리, `search_phase_node` 미션 FSM, depth_map/landing 노드 | Pi CPU | 고전 CV·제어 로직. NPU 대상 아님 |

## 구조 (2026-11 개정: 깡통 Pi 원커맨드 설치용)
```
Raspberry Pi OS Trixie 64-bit (호스트)
 ├─ Hailo-10H 드라이버/런타임  : apt  dkms + hailo-h10-all   (공식 지원 경로)
 ├─ hailo-server (systemd)     : HEF 로드, 127.0.0.1:5555 로 추론 제공 (ROS 없음)
 └─ Docker 컨테이너 kasa-ros   : ROS 2 Jazzy + px4_msgs + aruco_opencv + KASA 노드 (--net=host, --privileged)
      └─ hailo_yolo_inference_node --TCP--> hailo-server
```
ROS 바이너리는 Pi OS용이 없고 Hailo 바인딩은 Pi OS(시스템 파이썬)용만 공식 지원이라 둘을 분리했다.

## 깡통 Pi 설치 순서
1. (PC) Raspberry Pi Imager 로 **Raspberry Pi OS (64-bit, Trixie)** 설치. 고급 설정에서 호스트명/사용자/Wi-Fi/SSH 지정. AI HAT+ 2 를 Pi 5 에 장착(전원 끈 상태).
2. (PC, x86) `model_convert/` 로 `landing_pad.hef` 생성 → Pi 의 `/opt/kasa/landing_pad.hef` 로 복사 (scp 후 sudo mv).
3. (Pi) `sudo apt install -y git && git clone https://github.com/sw0812/NGD26.04.git ~/NGD26.04`
4. (Pi) `PX4_MSGS_BRANCH=<펌웨어 맞는 브랜치> bash ~/NGD26.04/26.04/hat/pi_deploy/bootstrap.sh` → 안내대로 `sudo reboot` → **같은 명령 한 번 더**
5. (Pi) `bash ~/NGD26.04/26.04/hat/pi_deploy/check_pi.sh` (전 항목 OK 확인) → `kasa-run mission | landing | all`

## 구성
- `model_convert/` (x86 PC): `export_onnx.py` → `make_calib.py` → `compile_hef.sh` → `landing_pad.hef`
- `kasa_hailo/` (ROS 2 패키지): 추론 클라이언트 노드, `hailo_landing.launch.py`, 카메라/ArUco 설정
- `pi_deploy/`: `bootstrap.sh`, `check_pi.sh`, `hailo_server/`(서버+systemd), `docker/Dockerfile`, `run_stack_pi.sh`

## 먼저 확인할 위험 요소
1. **Hailo 장치 인식**: 커뮤니티에 Trixie 에서 FW 로드 실패·구 `hailo_pci` 드라이버 충돌 보고가 있음. 안 뜨면 `dmesg | grep -i hailo`. `hailortcli` 는 H10 apt 패키지에 없을 수 있어 점검은 파이썬 `VDevice()` 로 함.
2. **컴파일**: `compile_hef.sh` 의 끝단 노드·NMS `convNN` 이름은 모델별로 다름. 학습 imgsz 와 export imgsz 불일치 주의(`best.pt` imgsz 는 torch 없이 못 읽음, 640 가정).
3. **NMS 출력**: `parse_nms()` 는 HailoRT 버전별 형식 가정 — 서버 로그의 `output=` shape 보고 수정. (서버는 출력 버퍼 1개 가정)
4. **정확도**: HEF 결과를 같은 이미지로 ONNX/PT 와 비교(INT8 손실).
5. **하드웨어**: 카메라 설정은 Jetson+C270 기준. `/dev/video0` 번호·YUYV 동작 확인. 전방 카메라는 두 번째 카메라가 있을 때만.
6. **px4_msgs**: 펌웨어 버전과 브랜치 일치 필수. XRCE-DDS 에이전트(`MicroXRCEAgent`)와 `ROS_DOMAIN_ID` 도 맞출 것 (현재 에이전트 설치/기동은 이 스크립트에 없음).
7. **성능**: 컨테이너(Jazzy on arm64) 위 ArUco+라인오도메트리+미션 노드가 카메라 30Hz 를 유지하는지 측정 필요.
8. `--privileged -v /dev:/dev` 는 편의상 넓게 열어둔 것. 안정화 후 `--device` 로 좁힐 것.
