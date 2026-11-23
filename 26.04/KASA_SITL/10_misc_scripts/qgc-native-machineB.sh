#!/usr/bin/env bash
# QGroundControl launcher (machine B, x86_64 Ubuntu 26.04).
# SDL inside QGC treats the RadioMaster Boxer as a gamepad and guesses the throttle
# (raw axis 2) as a one-sided "trigger" -> throttle stuck in the upper half / wrong axis.
# Force a symmetric mapping, AETR channel order, Mode 2 sticks:
#   left stick  X = CH4 rudder (a3),   Y = CH3 throttle (a2)
#   right stick X = CH1 aileron (a0),  Y = CH2 elevator (a1)
# QGC axis numbers then become: 0=leftx 1=lefty 2=rightx 3=righty.
# Same fix as machine A (qgc-docker.sh), same device GUID.
JOYSTICK_GUID="0300c19b09120000544f000011010000"
export SDL_GAMECONTROLLERCONFIG="${JOYSTICK_GUID},OpenTX Radiomaster Boxer Joystick,leftx:a3,lefty:a2,rightx:a0,righty:a1,platform:Linux,"
# Throttle bottom = 256/2047 on this radio -> make it the axis minimum (see boxer_abs_fix.py).
python3 -I "$HOME/QGroundControl/boxer_abs_fix.py"
exec "$HOME/QGroundControl/QGroundControl-x86_64.AppImage" "$@"
