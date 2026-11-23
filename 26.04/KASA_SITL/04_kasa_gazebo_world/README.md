# KASA 중급부문 「실내 조난자 탐색임무」 — Gazebo Classic 시뮬레이션 맵

규정집 기준으로 구성한 Gazebo Classic (gazebo11) 월드입니다. PX4 SITL과 함께 사용하는 것을 전제로 합니다.

## 구성 요소

모든 텍스처는 **동일 축척 200 px/m** 로 생성됩니다. 스크립트 실행 시 각 요소의 실측값과 규정값을 비교하는 검증표가 출력되며, 5 mm 이상 어긋나면 assert 로 중단됩니다.

| 요소 | 규정 | 구현 | 비고 |
|---|---|---|---|
| Safe area | 32 x 23 m | 32.000 x 23.000 m | 6400 x 4600 px |
| Mission area | 24 x 15 m | 24.000 x 15.000 m | safe area 중앙, 사방 4 m 여유 |
| Mission→Safe 여유 | 4 m | 4.000 m | 좌/우/상/하 동일 |
| 격자(구획선) | 3 x 3 m | 3.000 m | 8 x 5 셀, 내부 교차점 28개 |
| 구획선 폭 | 10 cm (TBR) | 0.100 m | 20 px |
| 경로점 ArUco | 50 x 50 cm | 0.500 m | ID 1~4, `DICT_5X5_50`, 100 px |
| 버티포트 | 지름 3 m, 높이 0.7 m | 3.000 m / 0.7 m | 원기둥 패드 (물리 충돌체 포함) |
| 항공기 안전구역 | 지름 2 m | 2.000 m | 패드 상판 빨간 링 |
| 버티포트 중앙 ArUco | 50 x 50 cm | 0.500 m | ID 0, 패드 상판 z=0.7 m |

버티포트 표식(외곽링 3 m + 안전구역링 2 m + V 픽토그램 + 중앙 ArUco)은 **패드 상판 텍스처**가 담당하고, 바닥 텍스처에는 위치 참고용 발자국 원만 옅게 표시됩니다. 실물처럼 마커가 0.7 m 높이에 있으므로 하향 카메라 고도 계산 시 유의하세요.

## 좌표계 — 버티포트 = 원점 (0,0,0)

규정이 *"GPS 신호가 수신되지 않는 실내 환경을 고려하여 **출발점 기준으로** 정확한 현재 위치를 계산"* 을 요구하므로, **출발점(버티포트)을 world 원점**으로 두었습니다.

이렇게 하면 세 좌표계가 전부 일치해서 변환이 필요 없습니다:

```
PX4 EKF local origin (= 스폰 지점)  ==  Gazebo world 원점  ==  채점 좌표계 원점
```

즉 `/mavros/local_position/pose` 값이 곧 규정이 말하는 "출발점 기준 현재 위치"이고, Gazebo ground truth와 직접 비교해 오차를 채점표(0.5 m / 2 m / 4 m) 기준으로 바로 평가할 수 있습니다.

| 항목 | 좌표 (ENU: x=East, y=North, z=Up) |
|---|---|
| 버티포트(출발점) | `(0, 0, 0)`, 상판 높이 `0.7 m` |
| **드론 스폰** | `(0, 0, 0.75)` — 상판 바로 위 |
| Mission area | x `[2 .. 26]`, y `[2 .. 17]` |
| Safe area | x `[-2 .. 30]`, y `[-2 .. 21]` |

**경로점 (예시 배치)** — `models/kasa_ground_field/waypoints.yaml` / `.json`

| ID | 좌표 | 버티포트로부터 |
|---|---|---|
| 1 | `( 8.0,  5.0)` | 9.43 m |
| 2 | `(11.0, 14.0)` | 17.80 m |
| 3 | `(17.0,  8.0)` | 18.79 m |
| 4 | `(23.0, 14.0)` | 26.93 m |

최적 탐색 순서 = ID 오름차순(1→2→3→4), 구조 복귀 순서 = 역순(4→3→2→1) — 규정 ⑥항.

실제 대회 배치는 현장에서 공지되므로 위 4개 좌표는 **연습/개발용 placeholder**입니다. `scripts/generate_field.py`의 `waypoints` 리스트를 고치고 재실행하면 됩니다(28개 교차점 중에서만 고를 수 있도록 assert가 걸려 있습니다).

> **고도 주의**: 버티포트 상판이 0.7 m 높이입니다. 드론이 상판에서 이륙하므로 PX4가 보고하는 상대고도 `z` 는 **상판 기준**입니다. Mission area 바닥 위 실제 고도는 `z + 0.75` 가 됩니다. 하향 카메라 시야폭 계산(구획선 검출)과 착륙 판정에 이 오프셋을 반영하세요.

> 원점을 safe area 중심으로 되돌리려면 `generate_field.py` 의 `ORIGIN_AT_VERTIPORT = False` 로 바꾸고 재실행하면 world 파일과 waypoints 가 함께 갱신됩니다.

## 실행 방법

### 가장 쉬운 방법 — 헬퍼 스크립트

```bash
./scripts/run_sitl.sh              # 맵만 실행 (드론 없음)
./scripts/run_sitl.sh iris         # PX4 SITL + iris 를 버티포트 위에 스폰
```

`PX4-Autopilot` 이 `~/PX4-Autopilot` 이 아니면 경로를 지정하세요:

```bash
PX4_DIR=~/src/PX4-Autopilot ./scripts/run_sitl.sh iris
```

스크립트가 환경변수 설정, 잔여 gazebo 프로세스 정리, 버티포트 위 `(0, 0, 0.75)` 스폰까지 처리합니다.

### 수동 실행

```bash
cd /path/to/kasa_gazebo_world
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/models
export GAZEBO_MODEL_DATABASE_URI=""      # 온라인 모델 DB 조회로 인한 hang 방지
gazebo --verbose worlds/kasa_indoor_search.world
```

> `GAZEBO_MODEL_PATH` 는 export 한 **그 터미널에서만** 유효합니다. 새 터미널을 열면 다시 지정하거나 `~/.bashrc` 에 추가하세요.
> 모델 폴더명(`kasa_ground_field`)이 곧 `model://` 이름입니다. 폴더명을 바꾸면 world 파일도 같이 고쳐야 합니다.

드론을 직접 스폰하려면 gzserver 기동 후:

```bash
gz model --spawn-file=$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf \
         --model-name=iris -x 0 -y 0 -z 0.75
```

PX4 버전에 따라 `make px4_sitl gazebo-classic_iris` 방식으로 world 를 넘기는 환경변수명(`PX4_SITL_WORLD` 등)이 다릅니다. 버전 차이를 타지 않으려면 위 `run_sitl.sh` 를 쓰는 편이 안전합니다.

### 문제 해결

| 증상 | 원인 / 해결 |
|---|---|
| 터미널이 멈춘 채 창이 안 뜸 | 온라인 모델 DB 조회 대기. `export GAZEBO_MODEL_DATABASE_URI=""` |
| `Unable to find uri[model://kasa_ground_field]` | `GAZEBO_MODEL_PATH` 미설정. export 한 터미널에서 그대로 실행할 것 |
| `Address already in use` | 이전 gazebo 프로세스 잔존. `pkill -9 -f gzserver; pkill -9 -f gzclient` |

## 재생성 / 커스터마이즈

```bash
pip install opencv-python pyyaml
python3 scripts/generate_field.py
```

`scripts/generate_field.py` 상단 파라미터를 바꾸면:

- 구획선 폭 (`LINE_W`, 10 cm 은 TBR — 확정되면 수정)
- ArUco 딕셔너리 (`ARUCO_DICT`, 현재 `DICT_5X5_50` — 대회 미확정 항목)
- 경로점 4개의 좌표 (`waypoints` 리스트)
- 버티포트 위치 (`vertiport`)
- 좌표계 원점 (`ORIGIN_AT_VERTIPORT`)

를 조정할 수 있고, 실행 한 번으로 **바닥 텍스처 · 버티포트 텍스처 · waypoints.yaml/json · world 파일**이 모두 함께 재생성됩니다. 좌표가 파일마다 어긋날 수 없는 구조입니다.

> `worlds/kasa_indoor_search.world` 는 자동 생성 파일입니다. 직접 수정하지 말고 스크립트를 고친 뒤 재실행하세요.

모든 도형은 미터 단위로 지정하고 `m2px()` 하나를 거쳐 그려지므로 축척이 갈라질 수 없으며, 실행 시 17개 항목의 실측/규정 비교표가 출력되고 5 mm 이상 어긋나면 assert 로 중단됩니다.

## 참고 — 아직 대회측 확인이 필요한 항목 (프로젝트 문서 6장 참고)

1. 구획선 폭 10cm 확정치
2. ArUco 딕셔너리 종류/ID 범위
3. 경로점 실제 배치 좌표(대회 당일 공지)
4. 좌표계 원점 정의
5. 바닥면 재질/색상, 조도 조건

이 시뮬레이션 맵은 규정집 수치를 기반으로 한 **개발/연습용**이며, 위 항목이 확정되면 스크립트 파라미터만 바꿔 즉시 다시 생성할 수 있도록 설계했습니다.
