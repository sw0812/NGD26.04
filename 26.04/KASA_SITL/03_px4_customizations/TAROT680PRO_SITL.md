# TAROT 680 Pro — PX4 SITL (Gazebo Classic) 모델 추가

PX4-Autopilot 저장소에 TAROT Iron Man 680 Pro 헥사콥터를 SITL 기체로 추가한다.
아래 수치는 **제조사 공개 스펙 + PX4 공식 소스 + Tarot ZYX-M 매뉴얼로 교차검증 완료**된 값이다.
임의로 바꾸지 말 것. 바꿔야 할 이유가 생기면 먼저 사용자에게 알릴 것.

---

## 0. 사전 확인

```bash
cd ~/PX4-Autopilot          # 경로 다르면 사용자에게 확인
git status --short          # 작업 전 더러운 상태인지 확인
```

모델 디렉토리 위치는 PX4 버전마다 다르다. 아래 순서로 **존재하는 것**을 찾아 쓴다.

1. `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models`
2. `Tools/simulation/gazebo/sitl_gazebo/models`
3. `Tools/sitl_gazebo/models`

에어프레임 디렉토리: `ROMFS/px4fmu_common/init.d-posix/airframes`

---

## 1. 기체 제원 (검증된 값)

| 항목 | 값 | 근거 |
|---|---|---|
| 프레임 | TL68P00, 모터간 695 mm, 780–810 g | Tarot 공식 / Flying Tech |
| 모터 | TL68P02 4006 620KV, 82 g, 126 mΩ | Tarot 공식 |
| 프롭 | 13×5.5 CF (TL2829) | — |
| **모터당 최대추력** | **1,580 g @ 17.4 A (4S)** | 벤더 실측 |
| 레퍼런스 AUW | 2.76 kg, 호버 14.5 분 | Tarot 공식 테스트 |
| 모델 AUW | **2.80 kg** | base_link 2.70 + 로터 6×0.017 |
| 암 반경 | **0.345 m** | 695/2 |
| T/W | 3.40 | |
| 호버 스로틀 | 0.542 | 실기 50–55% 스틱과 일치 |

### SDF 물리 파라미터

```
base_link  mass 2.70   ixx 0.062  iyy 0.062  izz 0.122     (Izz/Ixx = 1.97)
rotor_N    mass 0.017  ixx 9.75e-07  iyy 1.55e-04  izz 1.55e-04

motorConstant             4.05e-05     # T[N] = k·ω²  →  15.57 N @ 620 rad/s = 1587 g
maxRotVelocity            620          # rad/s ≈ 5921 rpm (620KV×14.8V 무부하 9176의 65%)
momentConstant            0.018        # Cq/Ct·D, 13인치
timeConstantUp/Down       0.02 / 0.04  # 4006+13×5.5는 무거워 10인치보다 느림
rotorDragCoefficient      1.0e-04
rollingMomentCoefficient  1.0e-06
rotorVelocitySlowdownSim  10
control_channels input_scaling  620    # maxRotVelocity와 반드시 동일
```

---

## 2. 로터 배치 — **절대 임의로 바꾸지 말 것**

PX4 공식 `6011_gazebo-classic_typhoon_h480` 및 Tarot ZYX-M 매뉴얼 Hexa-rotor X 도면과
대조 완료. 두 소스가 6개 각도 전부 일치한다.

| idx | 위치 | 각도(FRD) | 방향 | CA_ROTOR_PX | CA_ROTOR_PY | KM | SDF pose (x, y_left) |
|---|---|---|---|---|---|---|---|
| 0 | right | 90° | **cw** | 0.0 | 0.345 | −0.05 | `0 -0.345 0.06` |
| 1 | left | 270° | **ccw** | 0.0 | −0.345 | +0.05 | `0 0.345 0.06` |
| 2 | front-left | 330° | **cw** | 0.2988 | −0.1725 | −0.05 | `0.2988 0.1725 0.06` |
| 3 | rear-right | 150° | **ccw** | −0.2988 | 0.1725 | +0.05 | `-0.2988 -0.1725 0.06` |
| 4 | front-right | 30° | **ccw** | 0.2988 | 0.1725 | +0.05 | `0.2988 -0.1725 0.06` |
| 5 | rear-left | 210° | **cw** | −0.2988 | −0.1725 | −0.05 | `-0.2988 0.1725 0.06` |

주의할 점:

- **좌표계가 다르다.** 에어프레임 `CA_ROTOR*`는 PX4 FRD(x 전방, **y 우측**), SDF는 Gazebo
  ENU(x 전방, **y 좌측**). 그래서 같은 로터라도 y 부호가 반대다. 위 표에 둘 다 써뒀으니 그대로 쓸 것.
- **부호 규약:** `ccw ↔ KM +0.05`, `cw ↔ KM −0.05` (PX4 공식 파일에서 확인).
- 각도순 교차 배치: 30 ccw → 90 cw → 150 ccw → 210 cw → 270 ccw → 330 cw.
- ZYX-M 자체 모터 번호(M1=30°, M2=330°, M3=270°, M4=210°, M5=150°, M6=90°)는 PX4와 **다르다**.
  이 문서는 Pixhawk + PX4 전제. ZYX-M FC를 쓸 거면 PX4 SITL 자체가 맞지 않는다.

---

## 3. 모델 파일 생성

`<MODELS_DIR>/tarot680pro/` 를 만들고 `tarot680pro.sdf` + `model.config` 를 넣는다.
폴더명 = SDF 파일명이어야 한다.

SDF는 **`typhoon_h480` 모델을 베이스로 복제**한 뒤 아래를 교체하는 방식이 가장 안전하다
(플러그인 구성·센서 블록이 PX4 버전에 맞게 유지됨):

1. 모델 이름 `typhoon_h480` → `tarot680pro` 전부 치환
2. `base_link` 의 mass / inertia 를 §1 값으로 교체
3. 짐벌·카메라·랜딩기어 링크와 그 플러그인 전부 제거 (680 Pro에는 없음)
4. 로터 6개의 `<pose>` 와 `<turningDirection>` 을 §2 표대로 교체
5. 6개 모터 플러그인의 motorConstant / maxRotVelocity / momentConstant /
   timeConstant / drag 계수를 §1 값으로 교체
6. `mavlink_interface` 의 `control_channels` 를 6채널로, `input_scaling` 전부 620
7. visual/collision 은 단순 도형으로: 중앙 박스 `0.195 0.195 0.06`,
   암 6개 실린더(반경 0.008, 길이 0.345), 스키드 박스 2개 `0.30 0.02 0.02` @ z −0.18

**collision 에 STL 메시를 쓰지 말 것.** 삼각형 수만 개 충돌 검사로 시뮬이 느려진다.
외형용 STL은 visual 에만 선택적으로 붙인다.

---

## 4. 에어프레임 파일

`ROMFS/px4fmu_common/init.d-posix/airframes/6021_gazebo-classic_tarot680pro`
아래 내용 그대로 생성한다.

```sh
#!/bin/sh
#
# @name TAROT Iron Man 680 Pro Hexacopter (SITL)
#
# @type Hexarotor x
# @class Copter
#

. ${R}etc/init.d/rc.mc_defaults

param set-default MAV_TYPE 13

# Hexarotor X, arm radius 0.345 m, body frame FRD (x fwd, y right).
# Index order follows PX4's own hexa X convention (verified against the
# official 6011_gazebo-classic_typhoon_h480 airframe).
param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 6

param set-default CA_ROTOR0_PX   0.0
param set-default CA_ROTOR0_PY   0.345
param set-default CA_ROTOR0_KM  -0.05

param set-default CA_ROTOR1_PX   0.0
param set-default CA_ROTOR1_PY  -0.345
param set-default CA_ROTOR1_KM   0.05

param set-default CA_ROTOR2_PX   0.2988
param set-default CA_ROTOR2_PY  -0.1725
param set-default CA_ROTOR2_KM  -0.05

param set-default CA_ROTOR3_PX  -0.2988
param set-default CA_ROTOR3_PY   0.1725
param set-default CA_ROTOR3_KM   0.05

param set-default CA_ROTOR4_PX   0.2988
param set-default CA_ROTOR4_PY   0.1725
param set-default CA_ROTOR4_KM   0.05

param set-default CA_ROTOR5_PX  -0.2988
param set-default CA_ROTOR5_PY  -0.1725
param set-default CA_ROTOR5_KM  -0.05

# Gazebo Classic drives motors through the mavlink_interface plugin, so
# outputs use PWM_MAIN_FUNC — NOT SIM_GZ_EC_*, which is the gz-sim bridge.
param set-default PWM_MAIN_FUNC1 101
param set-default PWM_MAIN_FUNC2 102
param set-default PWM_MAIN_FUNC3 103
param set-default PWM_MAIN_FUNC4 104
param set-default PWM_MAIN_FUNC5 105
param set-default PWM_MAIN_FUNC6 106

# AUW 2.8 kg vs 6 x 1.58 kg max thrust -> T/W 3.4
param set-default MPC_THR_HOVER 0.54

# Starting gains for a 2.8 kg / 695 mm hexa. Tuning target, not verified.
param set-default MC_ROLLRATE_P  0.11
param set-default MC_ROLLRATE_I  0.22
param set-default MC_ROLLRATE_D  0.004
param set-default MC_PITCHRATE_P 0.11
param set-default MC_PITCHRATE_I 0.22
param set-default MC_PITCHRATE_D 0.004
param set-default MC_YAWRATE_P   0.18
param set-default MC_YAWRATE_I   0.10

param set-default MC_ROLL_P      6.0
param set-default MC_PITCH_P     6.0
param set-default MC_YAW_P       2.6

# Camera platform limits, not a racer.
param set-default MPC_XY_VEL_MAX 8.0
param set-default MPC_Z_VEL_MAX_UP 3.0
param set-default MPC_Z_VEL_MAX_DN 1.5
param set-default MPC_TILTMAX_AIR 35.0
param set-default MPC_LAND_SPEED 0.7
```

**넣지 말아야 할 것:** `SYS_CTRL_ALLOC`(현재 PX4에서 제거됨), `SIM_GZ_EC_*`(gz-sim 전용),
`PX4_GZ_MODEL` / `PX4_SIMULATOR` 환경변수(gazebo-classic 에어프레임엔 불필요).

---

## 5. 빌드 등록 (2곳, 둘 다 필요)

**(a) 에어프레임 등록** — `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`
의 `px4_add_romfs_files(` 목록에 `6021_gazebo-classic_tarot680pro` 추가.
`6011_gazebo-classic_typhoon_h480` 바로 아래가 자연스럽다.

**(b) 빌드 타겟 등록** — `typhoon_h480` 이 들어있는 cmake 파일을 찾아 `tarot680pro` 추가:

```bash
grep -rn "typhoon_h480" src platforms --include=*.cmake --include=CMakeLists.txt
```

보통 `src/modules/simulation/gazebo-classic/CMakeLists.txt` 의 `set(models ...)` 목록.
버전에 따라 `platforms/posix/cmake/sitl_target.cmake` 일 수도 있다.

---

## 6. 검증 (빌드 전에 반드시 수행)

```bash
# SDF XML 유효성
python3 -c "import xml.etree.ElementTree as ET; ET.parse('<MODELS_DIR>/tarot680pro/tarot680pro.sdf'); print('XML OK')"
```

그리고 아래 3자 일치를 스크립트로 확인할 것. 하나라도 어긋나면 멈추고 보고한다.

- SDF 로터 `pose` 로 계산한 각도 ↔ 에어프레임 `CA_ROTOR*_PX/PY` 로 계산한 각도
  (SDF는 y좌측이므로 `atan2(-y, x)`, CA는 y우측이므로 `atan2(PY, PX)`)
- SDF `turningDirection` ↔ `CA_ROTOR*_KM` 부호 (ccw↔+, cw↔−)
- 둘 다 §2 표의 기준값

물리 수치 역산도 확인:

```
k·ω²            = 4.05e-05 × 620²  = 15.57 N = 1587 g   (목표 1580 g, 오차 0.4%)
hover ω         = √(m·g/6/k) = 336 rad/s
hover throttle  = 336/620 = 0.542                        (MPC_THR_HOVER 0.54와 일치)
Izz/Ixx         = 0.122/0.062 = 1.97                     (평판형 멀티로터 이론 ~2.0)
```

---

## 7. 실행

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_tarot680pro
```

---

## 8. 증상별 대처

| 증상 | 원인 | 조치 |
|---|---|---|
| 타겟 없음 / `No rule to make target` | §5(b) 미등록 | 빌드 타겟 cmake 확인 |
| 기체 안 뜸, 에어프레임 못 찾음 | §5(a) 미등록 | airframes/CMakeLists.txt 확인 |
| arm 후 yaw 방향으로 계속 회전 | 방향↔KM 부호 꼬임 | §6 3자 일치 재검증 |
| 뜨자마자 뒤집힘 | 로터 인덱스 순서 꼬임 | §2 표대로 재배치 |
| 고주파 진동 | rate P 과다 | `MC_ROLLRATE_P` / `MC_PITCHRATE_P` 를 0.08까지 내려본다 |
| 둔하고 흔들림 | 질량 대비 게인 부족 | rate I 먼저 올린다 |
| 호버 스로틀이 0.54와 크게 다름 | 질량 또는 motorConstant 불일치 | §6 역산 재확인 |

PID 게인은 §1·§2와 달리 **검증된 값이 아니라 출발점**이다. 참고로 PX4 공식 typhoon은
`MC_ROLLRATE_P 0.08 / MC_ROLL_P 9` 를 쓴다.

---

## 9. 작업 완료 후 보고할 것

- 실제로 쓴 MODELS_DIR 경로와 빌드 타겟 cmake 파일 경로
- §6 3자 일치 검증 결과 (6개 인덱스 전부)
- 빌드 성공 여부, 실패 시 전체 에러 로그
- 임의 판단으로 §1·§2 수치를 바꿨다면 무엇을 왜 바꿨는지
