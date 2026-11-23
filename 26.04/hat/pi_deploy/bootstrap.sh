#!/usr/bin/env bash
# 완전 깡통 Raspberry Pi 5 + AI HAT+ 2(Hailo-10H) 원커맨드 설치. 미검증 초안 — 단계 출력 확인하며 실행.
# 전제: Raspberry Pi OS Trixie 64-bit (Imager 로 설치, 인터넷 연결, 일반 사용자 + sudo).
# 사용:  PX4_MSGS_BRANCH=release/1.16 bash bootstrap.sh        (처음 실행 후 재부팅 -> 같은 명령 한 번 더)
set -euo pipefail
: "${PX4_MSGS_BRANCH:?기체 PX4 펌웨어에 맞는 px4_msgs 브랜치를 지정 (예: PX4_MSGS_BRANCH=release/1.16)}"
REPO_URL=${REPO_URL:-https://github.com/sw0812/NGD26.04.git}
REPO=${REPO:-$HOME/NGD26.04}
STAMP=/var/lib/kasa_bootstrap_stage1

[ "$(uname -m)" = aarch64 ] || { echo "aarch64(64-bit OS) 필요"; exit 1; }
. /etc/os-release; [ "${VERSION_CODENAME:-}" = trixie ] || echo "!! Trixie 가 아님($VERSION_CODENAME): hailo-h10-all 이 없을 수 있음"

if [ ! -f $STAMP ]; then
  echo "== [1/2] 시스템 업데이트 + Hailo-10H 드라이버/런타임"
  sudo apt update && sudo apt -y full-upgrade
  sudo rpi-eeprom-update -a || true
  sudo apt install -y git dkms docker.io python3-numpy
  dpkg -l hailo-all 2>/dev/null | grep -q '^ii' && sudo apt -y purge hailo-all   # AI HAT+(1)용, H10 과 공존 불가
  sudo apt install -y hailo-h10-all
  # PCIe Gen3 (Hailo 권장) — 이미 있으면 건너뜀
  grep -q '^dtparam=pciex1_gen=3' /boot/firmware/config.txt || echo 'dtparam=pciex1_gen=3' | sudo tee -a /boot/firmware/config.txt >/dev/null
  sudo usermod -aG docker "$USER"
  sudo touch $STAMP
  echo; echo ">> 재부팅 후 같은 명령을 다시 실행하세요:  sudo reboot"; exit 0
fi

echo "== [2/2] 저장소 + 추론 서비스 + ROS 컨테이너"
[ -e /dev/hailo0 ] || { echo "!! /dev/hailo0 없음 — dmesg | grep -i hailo 확인 (HAT 장착/전원/FW 로드 오류)"; exit 1; }
[ -d "$REPO/.git" ] || git clone "$REPO_URL" "$REPO"
git -C "$REPO" pull --ff-only || true
H=$REPO/26.04/hat/pi_deploy

sudo mkdir -p /opt/kasa && sudo cp $H/hailo_server/hailo_server.py /opt/kasa/
sudo cp $H/hailo_server/hailo-server.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable hailo-server
[ -f /opt/kasa/landing_pad.hef ] && sudo systemctl restart hailo-server || \
  echo ">> /opt/kasa/landing_pad.hef 없음: x86 에서 컴파일한 HEF 를 복사한 뒤  sudo systemctl restart hailo-server"

sg docker -c "cd '$REPO' && docker build -f 26.04/hat/pi_deploy/docker/Dockerfile --build-arg PX4_MSGS_BRANCH=$PX4_MSGS_BRANCH -t kasa-ros ."

sudo tee /usr/local/bin/kasa-run >/dev/null <<EOS
#!/usr/bin/env bash
# kasa-run [mission|landing|all]  — ROS 컨테이너 실행 (host 네트워크, 카메라/로그 마운트)
exec docker run --rm -it --net=host --privileged -v /dev:/dev -v $REPO:/NGD26.04 -v \$HOME/logs:/root/logs \\
  -e ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-42} kasa-ros /usr/local/bin/run_stack_pi.sh "\${1:-mission}"
EOS
sudo chmod +x /usr/local/bin/kasa-run; mkdir -p $HOME/logs
echo; echo "설치 끝. 점검: bash $H/check_pi.sh   실행: kasa-run mission | landing | all"
