#!/usr/bin/env bash
# KASA 중급부문 맵 + PX4 SITL 실행 헬퍼
#
# 사용법:
#   ./scripts/run_sitl.sh                 # 맵만 실행 (드론 없음)
#   ./scripts/run_sitl.sh iris            # PX4 SITL + iris 를 버티포트 위에 스폰
#   PX4_DIR=~/PX4-Autopilot ./scripts/run_sitl.sh iris
#
# 버티포트 = world 원점 (0,0,0), 상판 높이 0.7 m
# 드론 스폰 = (0, 0, 0.75)  <- 상판 바로 위

set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# --- 공통 환경 ---------------------------------------------------------
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:${HERE}/models"
export GAZEBO_MODEL_DATABASE_URI=""        # 온라인 모델 DB 조회로 인한 hang 방지
WORLD="${HERE}/worlds/kasa_indoor_search.world"

# 버티포트 상판 위 스폰 좌표
SPAWN_X=0
SPAWN_Y=0
SPAWN_Z=0.75

echo "GAZEBO_MODEL_PATH = ${GAZEBO_MODEL_PATH}"
echo "WORLD             = ${WORLD}"

# 이전 세션 잔여 프로세스 정리
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
sleep 1

MODEL="${1:-}"

# --- 맵만 실행 ---------------------------------------------------------
if [ -z "${MODEL}" ]; then
    echo ">> 맵만 실행합니다 (드론 없음)"
    exec gazebo --verbose "${WORLD}"
fi

# --- PX4 SITL + 드론 ---------------------------------------------------
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
if [ ! -d "${PX4_DIR}" ]; then
    echo "ERROR: PX4-Autopilot 을 찾을 수 없습니다: ${PX4_DIR}"
    echo "       PX4_DIR=/경로/PX4-Autopilot $0 ${MODEL}  형태로 지정하세요."
    exit 1
fi

# PX4 의 gazebo-classic 모델도 경로에 추가
SITL_GZ="${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
[ -d "${SITL_GZ}" ] || SITL_GZ="${PX4_DIR}/Tools/sitl_gazebo"      # 구버전 경로
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:${SITL_GZ}/models"
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH}:${PX4_DIR}/build/px4_sitl_default/build_gazebo-classic"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${PX4_DIR}/build/px4_sitl_default/build_gazebo-classic"

echo ">> gzserver 기동"
gzserver --verbose "${WORLD}" &
GZ_PID=$!
sleep 4

echo ">> ${MODEL} 을 버티포트 위 (${SPAWN_X}, ${SPAWN_Y}, ${SPAWN_Z}) 에 스폰"
MODEL_SDF="${SITL_GZ}/models/${MODEL}/${MODEL}.sdf"
if [ ! -f "${MODEL_SDF}" ]; then
    # jinja 템플릿만 있는 경우 생성
    if [ -f "${SITL_GZ}/models/${MODEL}/${MODEL}.sdf.jinja" ]; then
        python3 "${SITL_GZ}/scripts/jinja_gen.py" \
            "${SITL_GZ}/models/${MODEL}/${MODEL}.sdf.jinja" "${SITL_GZ}" \
            --output-file "/tmp/${MODEL}.sdf"
        MODEL_SDF="/tmp/${MODEL}.sdf"
    else
        echo "ERROR: 모델 SDF 를 찾을 수 없습니다: ${MODEL_SDF}"
        kill ${GZ_PID}; exit 1
    fi
fi

gz model --spawn-file="${MODEL_SDF}" --model-name="${MODEL}" \
         -x ${SPAWN_X} -y ${SPAWN_Y} -z ${SPAWN_Z}

echo ">> gzclient (GUI) 기동"
gzclient --verbose &

echo ">> PX4 기동"
cd "${PX4_DIR}"
PX4_SIM_MODEL="${MODEL}" ./build/px4_sitl_default/bin/px4 \
    "${PX4_DIR}/ROMFS/px4fmu_common" \
    -s "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/rcS"

wait ${GZ_PID}
