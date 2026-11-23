#!/usr/bin/env bash
# 설치 점검 (각 항목 OK/FAIL)
ok(){ echo "OK   $1"; }; ng(){ echo "FAIL $1"; }
[ -e /dev/hailo0 ] && ok "/dev/hailo0" || ng "/dev/hailo0 (dmesg | grep -i hailo)"
python3 -c "from hailo_platform import VDevice; VDevice(); print('VDevice 생성')" 2>/dev/null && ok "hailo_platform + VDevice" || ng "hailo_platform/VDevice (드라이버·FW 버전 일치?)"
[ -f /opt/kasa/landing_pad.hef ] && ok "HEF 존재" || ng "/opt/kasa/landing_pad.hef 없음"
systemctl is-active --quiet hailo-server && ok "hailo-server 서비스" || ng "hailo-server (journalctl -u hailo-server -e)"
docker image inspect kasa-ros >/dev/null 2>&1 && ok "docker 이미지 kasa-ros" || ng "kasa-ros 이미지 없음"
ls /dev/video0 >/dev/null 2>&1 && ok "/dev/video0 카메라" || ng "카메라 없음"
