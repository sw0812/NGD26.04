#!/usr/bin/env bash
# 1회만 실행: PX4 빌드에 필요한 시스템/파이썬 패키지 설치 (sudo 필요)
set -e
cd "$(dirname "$0")"
sudo apt install -y python3-pip python3-venv ninja-build ccache cmake build-essential \
  libxml2-utils xsltproc protobuf-compiler libprotobuf-dev
python3 -m venv --system-site-packages .venv-px4
./.venv-px4/bin/pip install -r PX4-Autopilot/Tools/setup/requirements.txt
echo "설치 완료"
