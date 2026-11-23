# Gazebo Classic → gz-sim 이전 정리 (tarot680pro)

> 파일명: take.md (이전 이름 taken.md)

2026-11-10 작성. 비교 대상:
- Classic: `03_px4_customizations/airframes/6021_gazebo-classic_tarot680pro`, `models/tarot680pro/tarot680pro.sdf`
- gz-sim: `03_px4_customizations/airframes/6022_gz_tarot680pro`, `models_gz_sim/tarot680pro/model.sdf`

---

## 1. Classic에서는 됐는데 gz-sim에서 안 된 이유

### 증상 (gz-sim, 수정 전)
- 이륙 직후 요 각속도가 ±400°/s로 요동, 최대 3,594°/s → 결국 뒤집힘
  (로그: `PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-11-09/15_09_48.ulg`)

### 원인: `CA_ROTOR*_KM` 부호
6022(gz)는 6021(Classic)의 파라미터를 "그대로" 복사했는데, 그중 **KM 부호 전체 반전**이 Classic에서만 필요한 값이었음.

| 확인 항목 | 결과 |
|---|---|
| 모델 SDF (Classic vs gz) | 질량·관성·로터 위치·회전방향·모터상수 **완전히 동일** |
| PX4 공식 Classic typhoon (`6011`) KM | 문서 규약 (ccw = +0.05, cw = −0.05) |
| PX4 공식 gz x500 (`4001`) KM | 문서 규약 (ccw = +0.05, cw = −0.05) |
| 우리 6021 (Classic) KM | **반전** (2026-11-02 Classic 비행 테스트에서 이렇게 해야 회전이 멈췄음) |
| 우리 6022 (gz) KM, 수정 전 | 6021 복사 → **반전** → 요 발산 |
| 우리 6022 (gz) KM, 수정 후 | 문서 규약 → **2.5 m 호버, 요 각속도 ≈ 0 확인** (2026-11-10) |

### A/B 재검증 (2026-11-10, 같은 월드·같은 이륙 명령, PX4/Gazebo 재시작 후)

| 시간 | Classic KM (반전) | 문서 KM |
|---|---|---|
| 2초 | 요 0, 이륙 시작 | 요 0, 이륙 시작 |
| 6초 | 요 6.5 rad/s (374°/s), 2.5 m | 요 ≈ 0, 2.4 m |
| 10초 | 요 12.4 rad/s (711°/s) | 요 ≈ 0, 2.55 m 호버 |
| 12~18초 | 롤 34° → −155°, **뒤집힘** | 롤/피치 ≈ 2°, 안정 호버 |

→ gz-sim에서는 **문서 KM만 비행 가능**. 현재 6022는 문서 KM으로 되어 있음 (Classic 값 백업: `6022_gz_tarot680pro.bak_km`).

즉 **gz-sim에서는 문서(`TAROT680PRO_SITL.md` §2) 부호가 맞고**, 반전은 Classic 환경에서만 통하던 값.

### 미확인 사항 (솔직하게)
- Classic에서 **왜** 반전이 필요했는지는 확정 못 함. 공식 Classic typhoon도 문서 규약을 쓰고 SDF도 동일하므로,
  Classic 쪽 반전은 일반 규칙이 아니라 **Classic 환경의 다른 문제를 상쇄**하던 것으로 보임.
- Classic에서 roll/pitch/yaw rate P를 크게 올려야 했던 것(아래 §2-C)도 같은 원인과 관련 있을 수 있음 — 미검증.

---

## 2. Classic에서 가져와야 할 파라미터

### A. 그대로 가져올 것 (시뮬레이터와 무관한 기체 값) — 6022에 이미 반영됨

| 파라미터 | 값 | 비고 |
|---|---|---|
| `MAV_TYPE` | 13 | 헥사콥터 |
| `CA_AIRFRAME` | 0 | 멀티콥터 |
| `CA_ROTOR_COUNT` | 6 | |
| `CA_ROTOR0~5_PX`, `CA_ROTOR0~5_PY` | 문서 §2 | 로터 위치 (FRD) |
| `MPC_THR_HOVER` | 0.54 | |
| `MC_ROLLRATE_P/I/D` | 0.18 / 0.22 / 0.004 | Classic 튜닝값 (§C 참고) |
| `MC_PITCHRATE_P/I/D` | 0.18 / 0.22 / 0.004 | Classic 튜닝값 (§C 참고) |
| `MC_YAWRATE_I` | 0.10 | |
| `MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P` | 6.0 / 6.0 / 2.6 | |
| `MPC_XY_VEL_MAX` | 8.0 | |
| `MPC_Z_VEL_MAX_UP` / `MPC_Z_VEL_MAX_DN` | 3.0 / 1.5 | |
| `MPC_TILTMAX_AIR` | 35.0 | |
| `MPC_LAND_SPEED` | 0.7 | |
| `MPC_XY_VEL_D_ACC` | 0.35 | Classic QGC 수동비행으로 확인 |
| `EKF2_OF_CTRL` | 1 | 옵티컬 플로우 |
| `EKF2_RNG_CTRL` | 2 | 라이다 |
| `COM_ARM_WO_GPS` | 1 | |

### B. 가져오면 안 되는 것 (gz-sim용으로 바꿔야 함)

| Classic | gz-sim | 이유 |
|---|---|---|
| `CA_ROTOR0~5_KM` (반전값) | 문서 부호: 0:−0.05, 1:+0.05, 2:−0.05, 3:+0.05, 4:+0.05, 5:−0.05 | §1 참고. **다시 반전하지 말 것** |
| `PWM_MAIN_FUNC1~6` (101~106) | `SIM_GZ_EC_FUNC1~6` (101~106) | Classic은 mavlink_interface, gz는 gz_bridge 출력 |
| — | `SIM_GZ_EC_MIN1~6` = 150, `SIM_GZ_EC_MAX1~6` = 1000 | gz 전용 (PX4 gz 기체 공통값) |
| — | `SIM_GZ_EN` = 1 | gz 전용 |

### C. 가져왔지만 gz에서 다시 확인해야 할 것

| 파라미터 | 현재 | 문서 원래값 | 이유 |
|---|---|---|---|
| `MC_YAWRATE_P` | 0.6 | 0.18 | Classic에서 KM 반전 후 남은 요 흔들림을 잡으려고 올린 값. gz에서 KM이 정상이 된 지금은 과할 수 있음 |
| `MC_ROLLRATE_P`, `MC_PITCHRATE_P` | 0.18 | 0.11 | Classic에서 저감쇠(1.6 Hz 진동) 때문에 올린 값. gz에서 재검증 필요 |

### D. 사용자 결정 사항 (Classic과 다름)

| 파라미터 | Classic 6021 | gz 6022 기본값 | 현재 실행값 | 메모 |
|---|---|---|---|---|
| `EKF2_GPS_CTRL` | 0 | 3 | 3 | 2026-11-09 사용자 승인 (GPS 없으면 arm 불가) — 2026-11-10 airframe 파일에 반영 완료 |
| `COM_RC_IN_MODE` | (설정 안 함) | 1 | **1** | 4 = 스틱 무시(오프보드 전용), 1 = 조이스틱 전용. 조이스틱 테스트 목적, 2026-11-10 사용자 확인 후 airframe 파일에 반영 완료 |

---

## 3. 시뮬레이터별 다른 점 (파라미터 외)

| 항목 | Classic | gz-sim |
|---|---|---|
| 실행 | `make px4_sitl gazebo-classic_tarot680pro` | `make px4_sitl gz_tarot680pro` (`run_sitl.sh`) |
| 기체 스폰 | 월드 파일에 embed | `gz_bridge`가 런타임 스폰 (`PX4_SIM_MODEL`, `PX4_GZ_WORLD`, `PX4_GZ_MODEL_POSE`) |
| 모터 출력 | `PWM_MAIN_FUNC*` | `SIM_GZ_EC_*` |
| KM 부호 | 반전 (테스트로 확인) | 문서 규약 (테스트로 확인) |

---

## 4. 조종기(라디오마스터 박서) 인식/캘리브레이션 문제 — 해결법

> **머신 구분 (중요)**: 이 프로젝트는 두 대의 머신에서 작업됨. 아래 내용이 어느 머신 이야기인지 구분해서 읽을 것.
> - **머신 A — Jetson (aarch64, Ubuntu 20.04, glibc 2.31)**: gz-sim(신버전 Gazebo) 설치/빌드 불가 → Classic만 가능. QGC는 도커로 실행.
> - **머신 B — x86_64 PC (Ubuntu 26.04, ROS2 Lyrical, gz-sim10)**: gz-sim으로 `make px4_sitl gz_tarot680pro` **실제 실행·비행 확인됨** (2026-11-10). QGC AppImage 바로 실행됨.

## 4-A. 머신 A (Jetson, Ubuntu 20.04) 기준

> 머신 A에서 작성된 기록. 도커·glibc·SDL 관련 내용은 **머신 B에서는 검증 불가**(해당 환경이 없음).
> 스크립트는 머신 A의 `~/.local/bin/qgc-docker.sh`, USB 백업은 `10_misc_scripts/qgc-docker.sh`.

> 스크립트: `~/.local/bin/qgc-docker.sh` (QGC 실행기, 주석에 원인/해결 기록됨)

### 증상
QGroundControl에서 조종기(RadioMaster Boxer, USB joystick) 자체가 안 잡히거나,
잡혀도 캘리브레이션할 때 스로틀 축이 중간값 근처에 고정되어 끝까지 안 움직임.

### 원인 1: QGC AppImage가 호스트 glibc랑 안 맞음
Jetson(aarch64) 호스트는 Ubuntu 20.04 (glibc 2.31)인데, 최신 QGC AppImage는
glibc 2.38을 요구함 → 호스트에서 QGC 자체가 안 뜸.

**해결**: QGC를 `ubuntu:24.04` 도커 컨테이너 안에서 돌림 (`qgc-docker.sh`).

### 원인 2: 컨테이너가 조이스틱 하드웨어 접근 자체를 막음 (EPERM)
처음 만든 컨테이너(`qgc_container`)는 `/dev/input` 디바이스 cgroup이 안
열려있어서, 컨테이너 안 `ubuntu` 유저가 조이스틱을 하드웨어 레벨에서
`EPERM`으로 전혀 못 읽음.

**해결**: 컨테이너를 `--device-cgroup-rule='c 13:* rmw'` 옵션으로 새로
만듦 (`qgc_container2`). (13 = input 디바이스 major number)

### 원인 3: SDL이 스로틀 축을 "트리거"로 잘못 추측 → 캘리브레이션 중간값 고정
라디오마스터 박서가 SDL3 게임패드 매핑 DB에 없어서, SDL이 스로틀(axis2)을
"트리거" 축으로 잘못 추측함. 트리거 축은 0~1 편측(한쪽만) 정규화라서,
스틱을 아무리 움직여도 캘리브레이션이 항상 중간값 근처에 고정되는
버그가 생김.

**해결**: `SDL_GAMECONTROLLERCONFIG` 환경변수로 축 매핑을 직접 강제함 —
leftx/lefty/rightx/righty(전부 대칭 -1~1 정규화)로 명시 지정.
핵심은 `axis2 -> lefty` (스로틀을 트리거가 아니라 대칭축으로 강제).

```
JOYSTICK_GUID="0300c19b09120000544f000011010000"
export SDL_GAMECONTROLLERCONFIG="${JOYSTICK_GUID},OpenTX Radiomaster Boxer Joystick,leftx:a0,lefty:a2,rightx:a3,righty:a1,platform:Linux,"
```

### 참고: PX4 쪽 `COM_RC_IN_MODE`은 별개 설정
위 QGC/조이스틱 문제가 풀려도, PX4가 "조이스틱 입력을 쓸지"는 airframe의
`COM_RC_IN_MODE` 파라미터가 따로 결정함 (PX4 소스 `commander_params.yaml` 기준:
0=RC만, 1=MAVLink(조이스틱)만 — RC 관련 검사 꺼짐, 2=현재 소스가 끊기면 다른 소스로 전환,
3=**처음 유효했던 소스에 재부팅 전까지 고정**, 4=모든 입력 무시(오프보드 전용)).
2026-11-04 스톡 월드 테스트에서는 `3`으로 arm 확인됨. 6022(gz-sim)는 `1`(조이스틱 단독)로 반영, 사용자 확인 완료.

## 4-B. 머신 B (x86_64, Ubuntu 26.04) 기준 — 2026-11-10

4-A의 1~3(도커, cgroup, SDL 매핑)은 **필요 없었음**. QGC AppImage가 호스트에서 바로 실행되고,
박서는 `/dev/input/js0`으로 정상 인식(축 7개, 스로틀 최하단 ≈ −32700).

| 증상 | 원인 | 해결 | 검증 |
|---|---|---|---|
| arm 거부 `Resolve system health failures first` | `COM_RC_IN_MODE=3`이 이전 소스에 고정된 채 입력이 끊겨 `remote_control` 검사 실패 (`health_report` bit 7) | `COM_RC_IN_MODE=1` | ✅ 변경 후 `Preflight check: OK`, arm·비행 확인 |
| QGC 축 모니터에서 스로틀이 바닥 50 / 중앙 75 / 최상 100, 스틱 바닥인데 PX4엔 0 근처 | **4-A 원인 3과 동일**: QGC 안의 SDL이 Boxer를 게임패드로 인식해 축 번호를 게임패드식(leftx/lefty/rightx/righty/트리거)으로 재배열하고, 스로틀(raw a2)을 편측 '트리거'(QGC 축 4)로 추측 → 위쪽 절반만 사용. ini를 jstest 번호로 맞춰도 QGC 번호와 달라서 엉뚱한 축을 읽음 | `~/QGroundControl/qgc.sh`(백업: `10_misc_scripts/qgc-native-machineB.sh`)로 실행 — `SDL_GAMECONTROLLERCONFIG`로 AETR·Mode 2 강제(`leftx:a3,lefty:a2,rightx:a0,righty:a1`, GUID `0300c19b09120000544f000011010000` 머신 A와 동일). ini 축: 0=요, 1=스로틀(min −27272, max 32767, center 2747), 2=롤, 3=피치, `throttleModeCenterZero=false`. 또 `virtualJoystick=false`, `joystickEnabledVehiclesIds=1` 필요 | ✅ 2026-11-10 13:5x: 스로틀 바닥·나머지 중앙에서 PX4 `manual_control_setpoint` = throttle **−1.00**, roll/pitch/yaw 0.00, valid True (수정 전 −0.092). 스틱을 움직였을 때 방향(반전 여부)은 아직 미확인 |
| QGC에서 Enable joystick 켰는데 PX4에 입력이 안 감 (`manual_control_setpoint` 갱신 없음) | Enable joystick input이 꺼져 있거나 조종기 미연결 상태에서 확인했기 때문 | Joystick 탭에서 Enable joystick input 켜기 | ✅ 사용자 비행 로그에서 조이스틱 입력 유효 100%(`data_source=2`, MAVLink) 확인. ini의 `joystickEnabledVehiclesIds`가 비어 있어도 동작함 |

**2026-11-10 오후 — Joystick 탭 화면까지 확인 완료** (스크린샷: `10_misc_scripts/qgc_joystick_tab_verified_20261110.png`)
- 추가 원인: 조종기가 스로틀 맨 아래를 0이 아닌 **256 (범위 0~2047)** 으로 보냄(evdev ABS_Z). SDL은 0~2047 기준으로 환산해서
  바닥이 −32768이 아닌 약 −24572. QGC Joystick 탭의 Attitude Controls 슬라이더는 **보정값이 아니라 원시값**을 그림
  (QGC v5.1.5 `RemoteControlCalibrationController::_adjustChannelRawValue` — 반전만 적용) → ini 보정값을 바꿔도 슬라이더는 안 움직임.
- 조치: `~/QGroundControl/boxer_abs_fix.py`가 커널 absinfo의 ABS_Z 최솟값을 256으로 설정(EVIOCSABS). 조종기를 다시 꽂으면 초기화되므로
  `qgc.sh`가 QGC 실행 직전에 매번 실행. ini 스로틀 축(1번)은 min −32768 / max 32767 / center 0.
- 결과: 스로틀 바닥·나머지 중앙에서 Joystick 탭 Throttle 슬라이더 **왼쪽 끝**, Pitch/Roll/Yaw **중앙**, Raw Channel 2번 왼쪽 끝.
  Fly 화면에서 PX4 `manual_control_setpoint` throttle −1.00, roll/pitch/yaw 0.00, valid True.
- 참고: Joystick 설정 화면이 열려 있는 동안 QGC는 기체로 조종 입력을 보내지 않음(PX4 `valid: False`) — Fly 화면으로 나가면 정상.
- 스로틀 **최상단** 원시값(2047까지 가는지)은 아직 미확인 — 최상단에서 슬라이더가 오른쪽 끝에 안 닿으면 같은 방식으로 max도 맞출 것.

**2026-11-10 새벽 현재 QGC 설정** (`~/.config/QGroundControl/QGroundControl.ini`):
- (정정) QGC 축 번호는 SDL 게임패드 번호라 jstest 번호와 다름. 매핑은 위 표의 `qgc.sh` + ini 설정으로 확정.
  채널 순서는 AETR(CH1 에일러론, CH2 엘리베이터, CH3 스로틀, CH4 러더), Mode 2.
- `throttleModeCenterZero=true` (이전엔 false였음). RC 조종기 스로틀은 바닥이 0이어야 하므로 **false(Full down stick is zero throttle) 권장**.
- `joystickEnabledVehiclesIds=1`, `virtualJoystick=false` (가상 조이스틱이 켜져 있으면 Boxer 대신 가상 스틱 0값이 전송됨).

**남은 확인 순서** (조종기를 USB로 연결한 상태에서):
1. `jstest /dev/input/js0` — 스로틀 스틱을 움직일 때 바뀌는 **축 번호** 확인 → QGC 스로틀 축(4번)과 같은지 비교.
2. QGC Joystick 탭 → Throttle Mode "Full down stick is zero throttle", Enable joystick input 켜기.
3. `px4-listener manual_control_setpoint` — `timestamp`가 "0.0 seconds ago"로 계속 갱신, `valid: True`,
   스로틀 바닥 ≈ −1 / 최상 ≈ +1 인지 확인. 여기까지 되면 위 ❌ 두 줄을 ✅로 바꿀 것.

확인 명령 (pxh 또는 `build/px4_sitl_default/bin/px4-*`):
```
listener manual_control_setpoint   # throttle, valid, timestamp 확인
listener health_report             # arming_check_error_flags 비트 확인
param show COM_RC_IN_MODE
```

---

## 5. 자동 시뮬레이션 정상화 세션 (2026-11-10 새벽, 머신 B)

같은 시나리오를 매번 반복: 이륙 2.5 m → 호버 10초 → 요 90° → 북쪽 5 m 이동 → 요 0° → 착륙.
비교는 PX4 ulog + 시뮬레이터 실제 위치(`vehicle_local_position_groundtruth`) 기준.

### 5-1. 라이다가 기체 자기 카메라를 보고 있었음 (치명적, 수정함)
- 증상: 호버 30초쯤부터 기체가 실제로는 흘러가는데(실제 위치 −60 m) EKF는 제자리라고 판단 →
  맞는 GPS를 "글리치"로 거부 → 37초에 EKF가 80 m 점프 → 롤 93°, 위치 오차 수백 m.
- 원인: 하향 라이다(z=−0.1)가 바로 아래 `camera_down_link`(z=−0.25)의 2 cm 박스 **형상(visual)** 에 맞아
  고도와 상관없이 **0.14 m 고정**. `gpu_lidar`는 visual에 맞음. 플로우 속도 = 각속도 × 거리라서
  거리가 18배 작게 들어가 속도도 18배 작게 계산됨.
- 수정: `tarot680pro/model.sdf`에서 `camera_down_visual`(박스 형상)만 제거. 카메라 센서·위치·영상은 그대로.
  백업: `model.sdf.bak_lidar`(PX4 트리), USB `model.sdf.bak_1110`.
- **이전 메모의 "KASA 월드에서만 나는 `ESTIMATOR_GPS_GLITCH`(원인 미해결)"도 이 문제였을 가능성이 높음**
  (Classic 모델도 하향 카메라가 라이다 밑에 있었음 — Classic은 미검증).

### 5-2. 경기장 바닥이 단색이라 옵티컬 플로우가 0을 내면서 품질은 255로 보고 (부분 개선)
- 바닥 텍스처(6400×4600, 32×23 m)의 88%가 완전 단색 → 플로우 카메라(100×100, 화각 42°)에 특징점이
  거의 없음 → 플로우 0 출력. 그런데 플러그인 품질 = "찾은 특징점 중 추적 성공 비율"이라 2개만 찾아도 255.
  (콘솔의 `Number of good matches: 2, desired: 20`이 이것)
- 버티포트(무늬 있음) 위에서는 정상, 바닥 위에서만 문제.
- 추가 발견: 월드 조명이 어두워서 흰 바닥(250)이 카메라엔 회색(≈124)으로 찍힘 → 질감 대비도 절반 이하로 줄어듦.
  플로우 카메라 캡처로 확인(약한 질감: 영상 표준편차 2.5 = 사실상 단색).
- 시도 1: 부드러운 얼룩(±15) → 플로우 0 비율 약 50%. 시도 2: 각진 블록(±25) → 73%(대비 부족).
- **최종 조치: 흰/베이지 배경에만 각진 블록 무늬(20/10/4 cm, 흰 배경 ±약 50, 베이지는 0.7배) 추가.**
  선(회색·파랑·검정) 픽셀은 하나도 안 바뀜. 가장 어두운 블록도 흰 배경 196, 베이지 168로 회색 선(150)보다 밝음.
  원본 백업: `field_texture.png.orig_bak` (PX4 트리, `04_kasa_gazebo_world_gz_sim`, USB 모두).
- 결과: **바닥 위 플로우 0 비율 0%** (2회 연속 재현).

### 5-3. 결과 요약

| 항목 | 처음 | 라이다 수정 후 | + 약한 질감 | **+ 블록 질감 (최종, 2회)** |
|---|---|---|---|---|
| 라이다 거리 (2.5 m 호버) | 0.14 m 고정 | 2.5 m 정상 | 정상 | 정상 |
| 바닥 위 플로우 0 비율 | — | — | 약 50% | **0%** |
| 롤/피치 rate 오차 RMS | 66~74 °/s | 2.7~3.9 °/s | 0.7~1.0 °/s | 0.2~0.8 °/s |
| 요 rate 오차 RMS | 164 °/s | 10.7 °/s | 0.6 °/s | 0.6 °/s |
| 수평 위치 오차 RMS | 2.7~3.9 m (최대 82 m) | 0.6 m (최대 2.1 m) | 0.32~0.35 m (최대 1.5 m) | **0.05~0.17 m (최대 0.73 m)** |
| 고도 오차 RMS | 0.40 m | 0.10 m | 0.03 m | 0.03 m |
| 경고/고장 | Compass fault, 뒤집힘 | 없음 | 없음 | 없음 |

- 요 90° 회전: 오버슈트 없이 약 3초. → `MC_YAWRATE_P=0.6`은 gz에서도 문제없음, **변경 불필요**.
- 착륙: 정상 착지 → 자동 disarm, 자세 정상.

### 5-4. (선택) 남은 근본 문제 — 사용자 결정 사항
바닥 질감으로 실사용 문제는 해결됨. 다만 플로우 플러그인이 "특징점이 없을 때도 품질 255"를 낼 수 있는
근본 동작은 그대로라서, 무늬 없는 곳(다른 월드 등)에서는 재발 가능. 막으려면(메모 규칙상 임의로 안 바꿈):
- `EKF2_OF_QMIN`(기본 1) 올리기 — 품질 낮은 플로우를 버림. 단, 이 플러그인은 품질이 부풀려져 나와서 효과 제한적.
- `EKF2_OF_CTRL=0` — gz 바닥 위에서는 플로우를 끄고 GPS+라이다로만. (실내 GPS 없는 시나리오에는 부적합)
- 또는 플로우 플러그인 품질 계산을 "찾은 특징점 수 / 목표 20개"로 바꾸는 C++ 수정 (PX4 소스 변경).
