# Gazebo Classic → gz-sim 이전 정리 (tarot680pro)

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
| `EKF2_GPS_CTRL` | 0 | 3 | 3 | 2026-11-09 사용자 승인 (GPS 없으면 arm 불가) |
| `COM_RC_IN_MODE` | (설정 안 함) | 4 | **1** | 4 = 스틱 무시(오프보드 전용), 1 = 조이스틱 전용. 2026-11-10 조이스틱 테스트 위해 1로 변경 — **사용자 확인 필요** |

---

## 3. 시뮬레이터별 다른 점 (파라미터 외)

| 항목 | Classic | gz-sim |
|---|---|---|
| 실행 | `make px4_sitl gazebo-classic_tarot680pro` | `make px4_sitl gz_tarot680pro` (`run_sitl.sh`) |
| 기체 스폰 | 월드 파일에 embed | `gz_bridge`가 런타임 스폰 (`PX4_SIM_MODEL`, `PX4_GZ_WORLD`, `PX4_GZ_MODEL_POSE`) |
| 모터 출력 | `PWM_MAIN_FUNC*` | `SIM_GZ_EC_*` |
| KM 부호 | 반전 (테스트로 확인) | 문서 규약 (테스트로 확인) |
