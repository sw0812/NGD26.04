#!/usr/bin/env bash
# QGroundControl 실행 (Radiomaster Boxer 조이스틱 축 매핑 강제)
# SDL이 이 조종기를 모르고 스로틀(axis2)을 "트리거"(0~1 편측)로 잘못 추측해서
# 보정값이 중간에 고정되는 문제를 막기 위해 leftx/lefty/rightx/righty(대칭 -1~1)로 직접 지정.
JOYSTICK_GUID="0300c19b09120000544f000011010000"
export SDL_GAMECONTROLLERCONFIG="${JOYSTICK_GUID},OpenTX Radiomaster Boxer Joystick,leftx:a0,lefty:a2,rightx:a3,righty:a1,platform:Linux,"
exec "$HOME/QGroundControl/QGroundControl-x86_64.AppImage" "$@"
