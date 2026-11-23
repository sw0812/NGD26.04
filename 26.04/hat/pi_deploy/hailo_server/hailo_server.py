#!/usr/bin/env python3
"""호스트(Pi OS)에서 Hailo-10H 를 쥐고 있는 추론 서비스. ROS/torch 불필요 (apt: python3-h10-hailort).
프로토콜(localhost TCP): 연결 직후 서버가 헤더 '<II' (imgsz, 출력 float 개수) 전송.
  이후 요청 = imgsz*imgsz*3 바이트(RGB uint8, letterbox 완료) -> 응답 = 출력 버퍼 float32 바이트.
  (출력 = HEF 의 NMS 출력 그대로; 해석은 클라이언트 parse_nms)
사용: hailo_server.py /opt/kasa/landing_pad.hef [port]"""
import socket, struct, sys, numpy as np
from hailo_platform import VDevice, FormatType

hef = sys.argv[1]
port = int(sys.argv[2]) if len(sys.argv) > 2 else 5555

vdev = VDevice()
model = vdev.create_infer_model(hef)
model.input().set_format_type(FormatType.UINT8)
model.output().set_format_type(FormatType.FLOAT32)
in_shape, out_shape = tuple(model.input().shape), tuple(model.output().shape)
imgsz = in_shape[0]
out_len = int(np.prod(out_shape))
print(f'HEF={hef} input={in_shape} output={out_shape} (floats={out_len})', flush=True)
assert in_shape == (imgsz, imgsz, 3), f'정사각 RGB 입력 가정 위반: {in_shape}'

def recvn(c, n):
    b = bytearray()
    while len(b) < n:
        d = c.recv(n - len(b))
        if not d:
            raise ConnectionError
        b += d
    return bytes(b)

srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(('127.0.0.1', port)); srv.listen(1)
with model.configure() as cfg:
    print(f'listening 127.0.0.1:{port}', flush=True)
    while True:
        c, _ = srv.accept()
        c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            c.sendall(struct.pack('<II', imgsz, out_len))
            while True:
                img = np.frombuffer(recvn(c, imgsz * imgsz * 3), np.uint8).reshape(in_shape)
                b = cfg.create_bindings()
                b.input().set_buffer(np.ascontiguousarray(img))
                out = np.empty(out_shape, np.float32)
                b.output().set_buffer(out)
                cfg.run([b], 1000)
                c.sendall(out.tobytes())
        except (ConnectionError, OSError):
            c.close()
