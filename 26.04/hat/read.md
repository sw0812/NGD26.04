# hat 폴더 읽어보기 (Raspberry Pi 5 + AI HAT+ 2 / Hailo-10H 이식)

> 상태: **초안. 실제 Pi/HAT 에서는 아직 돌려보지 못함.** (문법·ROS 패키지 빌드만 확인)

## 한 줄 요약
드론 코드(ROS)는 Docker 상자 안에서, YOLO 추론(NPU)은 Pi OS 에서 돌리고 둘을 연결한다.
NPU 로 가는 건 YOLO 착륙패드 추론뿐이고, ArUco·라인 검출·미션 로직은 Pi CPU 에서 그대로 돈다.

## 폴더
- `model_convert/` : YOLO 모델(`best.pt`) → NPU 파일(`landing_pad.hef`) 변환. **x86 PC + Hailo Dataflow Compiler 필요**
- `kasa_hailo/`    : ROS 2 패키지 (NPU 추론 노드, launch, 카메라/ArUco 설정)
- `pi_deploy/`     : Pi 설치/실행 (`bootstrap.sh`, `check_pi.sh`, `run_stack_pi.sh`, `hailo_server/`, `docker/`)
- `README.md`      : 구조와 위험 요소 상세

## 설치 순서 (깡통 Pi)
1. PC: Raspberry Pi Imager 로 **Raspberry Pi OS 64-bit (Trixie)** 설치 (Wi-Fi/SSH 설정), HAT 장착
2. x86 PC: `model_convert/` 로 `landing_pad.hef` 생성 → Pi 의 `/opt/kasa/landing_pad.hef` 로 복사
3. Pi: `sudo apt install -y git && git clone https://github.com/sw0812/NGD26.04.git ~/NGD26.04`
4. Pi: `PX4_MSGS_BRANCH=<PX4 펌웨어 맞는 브랜치> bash ~/NGD26.04/26.04/hat/pi_deploy/bootstrap.sh`
   → 재부팅 안내가 나오면 `sudo reboot` → **같은 명령 한 번 더**
5. Pi: `bash ~/NGD26.04/26.04/hat/pi_deploy/check_pi.sh` (전부 OK 확인) → `kasa-run mission | landing | all`

## TODO / 알려진 문제
- [ ] `MicroXRCEAgent`(PX4↔ROS) 설치·기동 단계 없음
- [ ] `compile_hef.sh` 의 레이어 이름·NMS 설정은 모델에 맞게 수정 필요, 학습 imgsz(640 가정) 확인
- [ ] NMS 출력 파싱(`parse_nms`)은 첫 실행 로그 보고 확인
- [ ] INT8 양자화 후 정확도 비교 (ONNX vs HEF)
- [ ] HAT 인식 안 되면 `dmesg | grep -i hailo` (Trixie 에서 FW 로드 실패 보고 있음)
- [ ] Pi 에서 카메라 번호(`/dev/video0`), FPS 측정
