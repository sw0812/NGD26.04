#!/usr/bin/env python3
"""RadioMaster Boxer (EdgeTX USB joystick) reports throttle (ABS_Z) as 0..2047, but the
stick bottom only reaches 256. SDL (QGroundControl) scales by the reported min/max, so the
bottom shows up as ~-24572 instead of -32768 and QGC's Throttle slider never reaches the
left end. Narrow the kernel absinfo so 256 becomes the minimum. Resets when the radio is
re-plugged, so qgc.sh runs this before every QGC launch."""
import fcntl, glob, os, struct, sys
THROTTLE_CODE, NEW_MIN = 2, 256          # ABS_Z
for dev in sorted(glob.glob('/sys/class/input/event*/device/name')):
    if 'Radiomaster Boxer' not in open(dev).read():
        continue
    node = '/dev/input/' + dev.split('/')[4]
    fd = os.open(node, os.O_RDWR)
    buf = bytearray(24)
    fcntl.ioctl(fd, 0x80184540 + THROTTLE_CODE, buf)            # EVIOCGABS
    v, mn, mx, fuzz, flat, res = struct.unpack('6i', buf)
    fcntl.ioctl(fd, 0x401845c0 + THROTTLE_CODE,                  # EVIOCSABS
                struct.pack('6i', v, NEW_MIN, mx, fuzz, 0, res))
    fcntl.ioctl(fd, 0x80184540 + THROTTLE_CODE, buf)
    print(f'{node}: ABS_Z min {mn}->{struct.unpack("6i", buf)[1]}, max {mx}, value {v}')
    os.close(fd)
    sys.exit(0)
print('Boxer not connected - throttle range fix skipped')
