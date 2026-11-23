# KASA 드론 프로젝트 — 2026년 6월 ~ 11월 자료 백업

2026-11-09 작성, 2026-11-10 업데이트. tarot680pro 헥사콥터 PX4 SITL + ArUco 비전 + 라인트레이싱 미션 프로젝트의
모든 자료를 정리한 백업입니다. 아래 순서대로 읽으면 바로 이어서 작업할 수 있습니다.

**이 컴퓨터보다 사양이 낮은 새 컴퓨터로 옮기는 경우 읽는 순서**: 이 README → USB 루트의 `take.md`
(gz-sim 포팅 전체 히스토리 + 조종기 삽질 해결법, 아래 "최근 작업 로그" 섹션도 같이 참고) →
`12_memory_notes/`(확정된 기술 결론, 재조사 금지) → 그 다음에야 각 폴더 작업 시작. 새 환경에서
Claude(나)와 다시 작업한다면 이 세 가지만 먼저 읽히면 이전 대화 내용 없이도 바로 이어갈 수 있음.

## 폴더 구조

| 폴더 | 내용 |
|---|---|
| `01_ros2_ws_2604_MAIN/` | **Ubuntu 26.04 (ROS2 Lyrical Luth)용으로 마이그레이션 완료된 메인 워크스페이스.** 26.04에서는 이걸 쓰면 됨. 안의 `MIGRATION_NOTES.md` 참고. |
| `02_ros2_ws_foxy_backup/` | 원본 ROS2 Foxy 워크스페이스 (구버전 Ubuntu용, 비교/참고용 백업). |
| `03_px4_customizations/` | PX4-Autopilot 소스 전체가 아니라, **직접 수정/추가한 파일만** 모음 (airframe, sitl_run.sh, CMakeLists.txt, aruco_tracker.yaml, dataman, `models/tarot680pro/`(Gazebo Classic용 모델), `models_gz_sim/tarot680pro/`(**신버전 Gazebo(gz-sim)용으로 포팅한 모델, 2026-11-09 작업, 아래 참고**), `TAROT680PRO_SITL.md` 설치/셋업 문서). |
| `04_kasa_gazebo_world/` | KASA 실내 탐색 Gazebo **Classic**용 월드/모델 (vertiport_pad, aruco_test_marker, kasa_ground_field 등). |
| `04_kasa_gazebo_world_gz_sim/` | 같은 KASA 월드/모델을 **신버전 Gazebo(gz-sim)용으로 포팅**한 버전 (2026-11-09, 아래 tarot680pro 포팅 섹션 참고). |
| `05_kasa_project/` | `~/Desktop/kasa_project` 전체 (스테레오 뎁스, 라인 confidence, boundary detection 테스트 스크립트 등). |
| `06_vision_landing/` | ArUco 정밀착륙/비전 관련 코드 모음: `autonomous-precision-landing`, `aruco`(마커 생성), `offboard_test`, `Scripts`(모터/UART 테스트, attitude 3D viewer). |
| `07_flight_logs/` | PX4 SITL `.ulg` 로그(11/02~11/04) + 실기체 비행 로그(`flight_log_0929_*`) + `mav.parm`/`mav.tlog`. 디버깅 이력 재현용. |
| `08_datasets_models/` | YOLO 데이터셋 zip 3개, `yolov8n.pt/.onnx/.engine`, `ros_ws` 소스(카메라+YOLO 추론 노드). |
| `09_other_workspaces_src/` | `catkin_ws`, `ws`의 `src/`만 (빌드 산물 제외, 참고용). |
| `10_misc_scripts/` | 홈 디렉터리에 흩어져 있던 launch/퍼블리셔 스크립트들 (`my_launch.py`, `mavros_px4.launch.py`, `publisher.cpp`, `qgc-docker.sh`(QGroundControl을 도커로 실행 + 조종기 인식 문제 해결 스크립트, 아래 참고) 등). |
| `11_notes/` | 한글 파일명으로 흩어져 있던 메모를 알아볼 수 있게 이름 바꿔서 모음 (착륙 제어 흐름, 명령어 모음, YOLO 다음 단계, 튜닝 파라미터 등). |
| `12_memory_notes/` | 이번 작업 중 Claude가 정리해둔 **확정된(재조사 불필요) 기술 결론** 문서 2개. |
| `13_presentation/` | 무인기체계 심포지엄 발표자료 (`.pptx`). |

## 바로 이어서 하려면 (26.04 환경 기준)

1. ROS2 Lyrical Luth 설치 후 `01_ros2_ws_2604_MAIN/`을 `~/ros2_ws`로 복사.
2. `px4_msgs`, `ros2_aruco`는 **포함 안 됨** — 외부 git repo라 복사 대신 새로 clone해서 그 버전에 맞게 다시 빌드해야 함:
   ```
   git clone https://github.com/PX4/px4_msgs.git
   git clone https://github.com/<ros2_aruco repo>
   ```
3. `colcon build` — `01_ros2_ws_2604_MAIN/MIGRATION_NOTES.md`에 적힌 대로, 문법은 정적 스캔으로만 확인했고 실제 26.04 빌드 테스트는 이 환경에 Lyrical이 없어서 못 해봤음. 빌드 에러 나면 그 노트부터 확인.
4. PX4-Autopilot은 전체를 복사하지 않았음 (업스트림 소스, 수 GB) — `git clone https://github.com/PX4/PX4-Autopilot.git` 후 `03_px4_customizations/`의 파일들을 해당 경로에 덮어쓰기:
   - `airframes/*` → `ROMFS/px4fmu_common/init.d-posix/airframes/`
   - `sitl_run.sh.modified` → `Tools/simulation/gazebo-classic/sitl_run.sh`
   - `sitl_targets_gazebo-classic.cmake.modified` → `src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake`
   - `CMakeLists.txt.modified` → `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`

## tarot680pro 신버전 Gazebo(gz-sim) 포팅 (2026-11-09)

`03_px4_customizations/models_gz_sim/tarot680pro/model.sdf` + `03_px4_customizations/airframes/6022_gz_tarot680pro` —
Gazebo Classic용 tarot680pro 모델(질량/관성/로터 위치/모터상수/회전방향, 비행 테스트로 확인된 값)을
그대로 신버전 Gazebo(gz-sim) 문법(`MulticopterMotorModel` 플러그인, `<sensor>` 태그 기반 imu/mag/baro/navsat, `model://optical_flow`+`model://LW20` include)으로 포팅한 결과물.

**PX4 빌드 시스템에 정상 등록 확인함** (CMakeLists.txt에 airframe 추가, `make px4_sitl_default` 빌드 성공, ROMFS에 반영 확인).

**실제 실행은 이 머신에서 막혀있음**: `Tools/simulation/gz` 산하 신버전 Gazebo 브리지(`gz_bridge`)는 `gz-transport13/14/15`(Gazebo Harmonic 이후 신버전 패키지 네이밍)가 있어야 빌드되는데, 이 머신엔 `ignition-fortress`(구버전 네이밍, Classic 시뮬레이션이 쓰는 패키지)만 설치돼 있음. 그래서 `gz_x500`(PX4 기본 모델)조차 ninja 타겟이 안 생길 정도로 `gz_bridge` 모듈 자체가 꺼져있는 상태 — 우리 모델 파일 문제가 아니라 시스템 패키지 문제.

**이어서 하려면**: gz-harmonic(또는 그 이상) 패키지 설치 (`apt` 저장소 추가 필요, 기존 ignition-fortress와 공존 가능) 후 `touch src/modules/simulation/gz_bridge/CMakeLists.txt && make px4_sitl_default`로 재구성, 그 다음 `make px4_sitl gz_tarot680pro`로 실행. 사용자 확인 결과 지금 당장 설치는 보류하고 파일만 준비해둔 상태.

### 맵(KASA 월드)도 같은 날 포팅함

`04_kasa_gazebo_world_gz_sim/` — `kasa_indoor_search.world`(Classic)를 `kasa_indoor_search.sdf`(gz-sim)로, 그리고 거기서 쓰는 3개 모델(`kasa_ground_field`, `vertiport_pad`, `aruco_test_marker`)을 전부 gz-sim 문법(머티리얼: Ogre 스크립트 → PBR albedo_map)으로 포팅함. 좌표계/미션 영역/경로점(ArUco ID1~4) 배치는 Classic과 동일.

**Classic과 다르게 처리한 부분 (의도적 설계 변경, 단순 포팅 아님)**: Classic 월드는 tarot680pro를 월드 파일에 직접 embed했음(스폰 방식이 "모델의 mavlink 플러그인이 gzserver 기동 후 알아서 접속"이라 그렇게 해야 GPS 글리치를 피할 수 있었음). gz-sim은 스폰 방식이 다름 — PX4의 `gz_bridge`가 `PX4_SIM_MODEL` 환경변수로 런타임에 EntityFactory 서비스를 호출해서 스폰하는 구조라, 월드에 또 embed하면 중복 스폰/이름 충돌 가능성이 높음. 그래서 **embed 안 하고**, 대신 airframe 파일(`6022_gz_tarot680pro`)의 `PX4_GZ_WORLD` 기본값을 `kasa_indoor_search`로 바꿔서 런타임 스폰이 이 월드를 타겟하게 해놨음. GPS 글리치가 이 스폰 방식에서도 재현되는지는 **미검증** (gz-sim 자체를 못 띄워봄) — 재현되면 그때 가서 embed를 다시 넣는 게 맞음, 선제적으로 복붙하지 않음.

PX4 빌드(`make px4_sitl_default`)는 이 월드/모델 추가 후에도 정상 통과 확인함. 단, **이것도 역시 gz-transport 패키지가 없어서 실제 실행 검증은 못 했음** — 위 블로커와 동일.

### [머신 A] gz-sim 설치 검토 결과 (2026-11-09) - 머신 A에서는 설치 불가, Classic으로만 진행 확정 (머신 B는 gz-sim 실행됨, 아래 11-10 로그 참고)

실제로 설치 가능한지 조사함:
- 이 머신: **Ubuntu 20.04(Focal) / arm64**.
- OSRF 공식 저장소에 Focal+arm64용으로 올라온 신버전 Gazebo는 **Garden(gz-sim7 / gz-transport12)까지뿐** — Harmonic(gz-transport13+)은 Focal 패키지가 애초에 없음(OSRF가 Harmonic부터 Focal 지원 중단).
- 반대로 이 PX4 트리(v1.18.0-beta1)의 `gz_bridge` 모듈은 `find_package(gz-transport NAMES gz-transport gz-transport13 gz-transport14 gz-transport15)`로 **Harmonic 이상만 받음** — Garden(12)은 업스트림에서 의도적으로 뺀 것.
- 즉 Garden을 억지로 설치해도 PX4 쪽 CMake 버전 체크를 못 넘고, 체크를 패치해서 넘겨도 GZBridge 소스가 Harmonic 전용 API를 쓸 가능성이 있어 컴파일 에러/이상동작 위험이 있음.
- **결론 (사용자 확인 완료)**: 지금 이 머신에서 신버전 Gazebo 설치는 보류, Classic 시뮬레이션으로만 계속 작업. 위에서 만든 gz-sim 포팅 파일(`models_gz_sim/`, `04_kasa_gazebo_world_gz_sim/`)은 **나중에 Ubuntu 22.04+ 머신으로 옮기거나 OS를 업그레이드하면 그때 그대로 쓸 수 있게** 보관용으로 남겨둔 것.

## 2026-11-10 작업 로그 (가제보6/Fortress 조사 + KM 버그 + 조종기 해결법)

> **머신 구분 (중요)**: 이 프로젝트는 두 대의 머신에서 작업됨. 아래 내용이 어느 머신 이야기인지 구분해서 읽을 것.
> - **머신 A — Jetson (aarch64, Ubuntu 20.04, glibc 2.31)**: gz-sim(신버전 Gazebo) 설치/빌드 불가 → Classic만 가능. QGC는 도커로 실행.
> - **머신 B — x86_64 PC (Ubuntu 26.04, ROS2 Lyrical, gz-sim10)**: gz-sim으로 `make px4_sitl gz_tarot680pro` **실제 실행·비행 확인됨** (2026-11-10). QGC AppImage 바로 실행됨.

**[머신 A 기준]** 위 "gz-sim 설치 검토"(11-09, Harmonic/Garden 불가 확정) 다음날, 사용자가 "이미 설치된
gazebo6(Fortress)로 해보자"고 지시해서 추가 조사함.

### Fortress("gazebo6")도 안 됨 — 결론
- Fortress는 `ignition-gazebo6`/`ignition-transport11` 구버전 네이밍 패키지. OSRF가 올려둔
  "gz-"로 시작하는 작은(10KB) 패키지들은 진짜 신버전이 아니라 구버전 패키지로 연결만 해주는
  **빈 호환용 패키지**라서 PX4가 찾는 `find_package(gz-transport ...)`를 못 채워줌.
- `ignition-gazebo6`는 `/usr/include/ignition/gazebo6/gz/sim/...` 경로로 신버전 네임스페이스
  (`gz::sim`) 헤더를 같이 포함하고 있어서 반쪽은 호환되는데, **`ignition-transport11`은 이 호환
  레이어가 전혀 없음** — `gz::transport` 네임스페이스 자체가 없어서 PX4 소스가 아예 못 찾음.
- 즉 Harmonic이든 Garden이든 Fortress든, 이 머신(Ubuntu 20.04/arm64)에 설치 가능한 어떤
  Gazebo 버전으로도 PX4 v1.18의 `gz_bridge` 빌드 게이트(`find_package(gz-transport ...)`)를
  못 넘음. 억지로 하려면 PX4 CMake에 수동 패치 + 플러그인 파일명(`gz-sim-*` → `ignition-gazebo6-*`)
  전부 바꿔야 하는데, 성공 보장 없는 큰 작업이라 사용자에게 보고 후 **보류, Classic으로만 진행 유지**.

### tarot680pro gz-sim 포팅 모델에서 KM 부호 버그 발견 + 수정
`take.md`(USB 루트)에 전체 내용 있음 — 요약:
- **[머신 B에서 실제 비행으로 확인]** gz-sim 포팅 당시 `6022_gz_tarot680pro`의 `CA_ROTOR*_KM`을
  Classic 값(`6021`, 부호 전부 반전)에서 그대로 복사했음. 그대로 두면 이륙 직후 요(yaw)가 발산해서 뒤집힘
  (A/B 테스트 2회: Classic 부호 → 10초에 711°/s, 18초에 뒤집힘 / 문서 부호 → 2.5 m 안정 호버).
- **확인된 사실**: Classic·gz 두 SDF는 로터 배치·회전방향·모터상수까지 완전히 동일. PX4 공식 기체는
  Classic(typhoon `6011`)과 gz(`x500`) **둘 다** 문서 부호(ccw=+0.05, cw=−0.05)를 씀.
- **미확인**: Classic에서 왜 반전이 필요했는지는 모름. "두 플러그인이 `turningDirection`을 다르게
  해석한다"는 설명은 공식 Classic typhoon이 문서 부호로 정상 비행한다는 점과 맞지 않아 **검증 안 된 가설**임.
  Classic 쪽 반전은 Classic 환경의 다른 문제를 상쇄하던 것일 가능성이 있음.
- PX4 공식 문서 규약(ccw=+0.05, cw=-0.05, 공식 typhoon/gz_x500과 동일)으로 부호를 되돌려서 해결.
  `6022_gz_tarot680pro` 파일 수정 완료, USB에도 동기화됨.

### GPS/조종기 파라미터 확정 (사용자 확인 완료, 2026-11-10)
- `EKF2_GPS_CTRL=3` (스톡 월드에서 arm 확인된 값), `COM_RC_IN_MODE=1`(조이스틱 단독 테스트용) —
  둘 다 `6022_gz_tarot680pro`에 반영, USB 동기화 완료. (6021 Classic 쪽은 그대로 `GPS_CTRL=0` 유지,
  사용자가 명시적으로 그렇게 지시해서 안 건드림.)

### 조종기(라디오마스터 박서) 인식 문제 — 3단계 삽질과 해결법
**[머신 A 기준]** `take.md` §4에 전체 기록, 실제 스크립트는 `10_misc_scripts/qgc-docker.sh`.
머신 B(26.04)에서는 아래 1~3이 필요 없었음 — 머신 B 쪽 원인/해결은 `take.md` §4-B 참고:
1. QGC AppImage가 요구하는 glibc 2.38을 이 머신(Ubuntu 20.04, glibc 2.31)이 못 채움 →
   `ubuntu:24.04` 도커 컨테이너 안에서 QGC 실행.
2. 처음 만든 컨테이너는 `/dev/input` 디바이스 cgroup이 안 열려서 조이스틱이 `EPERM`으로
   하드웨어 레벨에서 안 읽힘 → `--device-cgroup-rule='c 13:* rmw'` 옵션으로 컨테이너 재생성.
3. SDL이 박서의 스로틀 축을 "트리거"로 잘못 추측해서 캘리브레이션이 중간값에 고정되는 버그 →
   `SDL_GAMECONTROLLERCONFIG` 환경변수로 `axis2→lefty`(대칭 -1~1 정규화) 강제 매핑해서 해결.

## 지금 멈춰있는 지점 (12_memory_notes 요약)

- **GPS 없이 arm은 PX4 EKF2 구조상 불가능**: `EKF2_GPS_CTRL=0`으로 두면 어떤 비행 모드에서도 arm이 안 됨 — 파라미터 문제가 아니라 EKF2 소스(`estimatorCheck.cpp`/`optical_flow_control.cpp`/`range_height_control.cpp`/`terrain_control.cpp`)의 순환 부트스트랩 구조 문제로 확인됨 (EKF2 코드 수정 없이는 해결 불가). **`6021_gazebo-classic_tarot680pro`(Classic)는 사용자 지시대로 `EKF2_GPS_CTRL=0` 유지 중(그 상태로는 arm 안 됨, 의도된 상태). `6022_gz_tarot680pro`(gz-sim 포팅)는 2026-11-10 사용자 확인 후 `EKF2_GPS_CTRL=3`으로 반영해서 arm 가능한 상태**(위 "2026-11-10 작업 로그" 참고) — 둘이 값이 다르니 헷갈리지 말 것.
- KASA 커스텀 월드(`04_kasa_gazebo_world`)에서만 재현되는 `ESTIMATOR_GPS_GLITCH`는 원인 미해결 (모델/스폰 방식 문제는 아님을 A/B 테스트로 확인).
- `aruco_test_marker`가 카메라 피드에 안 보이는 문제 미해결 (plane/box/RGB 다 시도, 보류 상태).
- `line_tracker.py`/`search_phase_node.py` (`01_ros2_ws_2604_MAIN/src/my_first_pkg/`): topic 버전 수정(`vehicle_status_v1`→`v4`)만 하고 end-to-end 실행 테스트는 안 해봄.

## 제외한 것

- `id` 파일 — 평문 비밀번호가 들어있어서 보안상 USB에 포함하지 않음. 별도 비밀번호 관리 도구로 옮기는 걸 권장.
- PX4-Autopilot 전체 소스/빌드 산물 (3.5GB+, 업스트림 git repo라 clone이 더 정확함).
- `sitl_gazebo-classic.zip` (77MB, 빌드 산물 백업 — 필요하면 다시 빌드).
- `qgroundcontrol` 전체 (1.7GB, 업스트림 QGC 소스 클론 — 커스터마이징 없음).
