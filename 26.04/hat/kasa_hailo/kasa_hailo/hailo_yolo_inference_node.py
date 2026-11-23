#!/usr/bin/env python3
"""yolo_inference_node.py 의 Hailo-10H 버전 (컨테이너 ROS 노드 = 소켓 클라이언트).
HailoRT 는 호스트(Pi OS)의 hailo_server(pi_deploy/hailo_server)가 쥐고 있고, 이 노드는 letterbox 한 이미지를
localhost TCP 로 보내 NMS 결과를 받는다 (ROS 컨테이너 파이썬과 Hailo 바인딩 파이썬 버전이 달라도 되도록).
ultralytics/torch 추론만 NPU 추론으로 교체했고, depth 렌더링 -> 3D 역투영 -> NED 변환 ->
/landing_target 발행 로직은 원본(01_ros2_ws_2604_MAIN/.../yolo_inference_node.py)과 동일하다.

전제: HEF를 NMS 후처리 포함(nms_postprocess)으로 컴파일 (compile_hef.sh). 입력 imgsz x imgsz RGB uint8,
      학습과 같은 letterbox. 출력 = 클래스별 [y_min, x_min, y_max, x_max, score] (0~1 정규화).
※ 실기기 미검증 초안: HailoRT 버전에 따라 NMS 출력 버퍼 형태가 다를 수 있어, 시작 시 출력 shape를
  로그로 찍는다. 다르면 parse_nms() 만 고치면 된다.
"""
import math
import socket
import struct
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import VehicleLocalPosition

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)


class HailoYoloInferenceNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_node')
        P = self.declare_parameter
        P('server_host', '127.0.0.1'); P('server_port', 5555)
        P('imgsz', 640)
        P('depth_topic', '/depth/image')
        P('local_pos_topic', '/fmu/out/vehicle_local_position_v1')
        P('output_topic', '/landing_target')
        P('conf_threshold', 0.4)
        P('h_samples', 360); P('v_samples', 64); P('v_min_deg', -50.0); P('v_max_deg', 50.0)
        P('render_min', 0.5); P('render_max', 12.0)
        P('lidar_offset_x', 0.0); P('lidar_offset_y', 0.0); P('lidar_offset_z', -0.05)
        g = lambda k: self.get_parameter(k).value

        self.imgsz = int(g('imgsz'))
        self.conf_th = float(g('conf_threshold'))
        self.h_samples, self.v_samples = int(g('h_samples')), int(g('v_samples'))
        self.v_min, self.v_max = math.radians(g('v_min_deg')), math.radians(g('v_max_deg'))
        self.render_min, self.render_max = float(g('render_min')), float(g('render_max'))
        self.lidar_off = (float(g('lidar_offset_x')), float(g('lidar_offset_y')), float(g('lidar_offset_z')))

        # --- hailo_server 연결 (헤더: imgsz, 출력 float 개수) ---
        self.sock = None
        self.host, self.port = g('server_host'), int(g('server_port'))
        self.connect()

        self.drone_x = self.drone_y = self.drone_z = self.drone_yaw = 0.0
        self.have_local_pos = False
        qos_img = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Image, g('depth_topic'), self.depth_cb, qos_img)
        self.create_subscription(VehicleLocalPosition, g('local_pos_topic'), self.local_pos_cb, PX4_QOS)
        self.target_pub = self.create_publisher(PointStamped, g('output_topic'), 10)

    # ---------- 원본과 동일 ----------
    def local_pos_cb(self, m):
        self.drone_x, self.drone_y, self.drone_z = m.x, m.y, m.z
        self.drone_yaw = getattr(m, 'heading', 0.0)
        self.have_local_pos = True

    def render_depth_image(self, depth):
        clipped = np.clip(depth, self.render_min, self.render_max)
        img = ((self.render_max - clipped) / (self.render_max - self.render_min) * 255.0).astype(np.uint8)
        img[depth <= 1e-6] = 0
        return img

    def pixel_to_depth_value(self, depth, col, row):
        r0, c0 = int(round(row)), int(round(col))
        patch = depth[max(0, r0 - 1):min(depth.shape[0], r0 + 2), max(0, c0 - 1):min(depth.shape[1], c0 + 2)]
        valid = patch[patch > 1e-6]
        return float(np.median(valid)) if valid.size else None

    def pixel_to_lidar_xyz(self, col, row, d):
        az = (col + 0.5) / self.h_samples * 2 * math.pi - math.pi
        el = self.v_min + (row + 0.5) / self.v_samples * (self.v_max - self.v_min)
        hr = d * math.cos(el)
        return hr * math.cos(az), hr * math.sin(az), d * math.sin(el)

    def lidar_to_ned(self, xl, yl, zl):
        xb, yb, zb = xl + self.lidar_off[0], yl + self.lidar_off[1], zl + self.lidar_off[2]
        xf, yf, zf = xb, -yb, -zb
        c, s = math.cos(self.drone_yaw), math.sin(self.drone_yaw)
        return (self.drone_x + xf * c - yf * s, self.drone_y + xf * s + yf * c, self.drone_z + zf)

    # ---------- Hailo 서버 통신 ----------
    def _recvn(self, n):
        buf = bytearray()
        while len(buf) < n:
            c = self.sock.recv(n - len(buf))
            if not c:
                raise ConnectionError('hailo_server 연결 끊김')
            buf += c
        return bytes(buf)

    def connect(self):
        while rclpy.ok():
            try:
                self.sock = socket.create_connection((self.host, self.port), timeout=5)
                self.sock.settimeout(2.0)
                srv_imgsz, self.out_len = struct.unpack('<II', self._recvn(8))
                if srv_imgsz != self.imgsz:
                    self.get_logger().warn(f'서버 imgsz={srv_imgsz} 로 맞춤 (파라미터 {self.imgsz})')
                    self.imgsz = srv_imgsz
                self.get_logger().info(f'hailo_server 연결: imgsz={self.imgsz} 출력 float {self.out_len}개')
                return
            except OSError as e:
                self.get_logger().warn(f'hailo_server 대기 중 ({e})'); self.sock = None; time.sleep(2.0)

    # ---------- Hailo 전용 ----------
    def letterbox(self, img):
        h, w = img.shape[:2]
        s = self.imgsz / max(h, w)
        nh, nw = int(round(h * s)), int(round(w * s))
        top, left = (self.imgsz - nh) // 2, (self.imgsz - nw) // 2
        canvas = np.full((self.imgsz, self.imgsz, 3), 114, np.uint8)
        canvas[top:top + nh, left:left + nw] = cv2.resize(img, (nw, nh))
        return canvas, s, left, top

    def parse_nms(self, buf):
        """NMS(by class) 플랫 버퍼: 클래스마다 [count, (y0,x0,y1,x1,score)*count]. nc=1 가정.
        반환: [(score, cx, cy)] (letterbox 입력 픽셀 기준)"""
        a = np.asarray(buf, dtype=np.float32).reshape(-1)
        n = int(a[0]); dets = []
        for i in range(n):
            y0, x0, y1, x1, sc = a[1 + 5 * i: 6 + 5 * i]
            dets.append((float(sc), (x0 + x1) / 2 * self.imgsz, (y0 + y1) / 2 * self.imgsz))
        return dets

    def detect(self, render_gray):
        rgb = cv2.cvtColor(render_gray, cv2.COLOR_GRAY2RGB)
        inp, s, left, top = self.letterbox(rgb)
        try:
            self.sock.sendall(np.ascontiguousarray(inp).tobytes())
            out = np.frombuffer(self._recvn(self.out_len * 4), dtype=np.float32)
        except (OSError, ConnectionError) as e:
            self.get_logger().warn(f'추론 실패({e}), 재연결'); self.connect(); return None
        dets = [d for d in self.parse_nms(out) if d[0] >= self.conf_th]
        if not dets:
            return None
        sc, cx, cy = max(dets)
        return sc, (cx - left) / s, (cy - top) / s       # 원본 360x64 픽셀 좌표로 복원

    def depth_cb(self, msg):
        if not self.have_local_pos:
            return
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
        det = self.detect(self.render_depth_image(depth))
        if det is None:
            self.get_logger().info('Hailo 검출 없음'); return
        conf, cx, cy = det
        d = self.pixel_to_depth_value(depth, cx, cy)
        if d is None:
            self.get_logger().warn('검출됐으나 유효 depth 없음, 스킵'); return
        tx, ty, tz = self.lidar_to_ned(*self.pixel_to_lidar_xyz(cx, cy, d))
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg(); out.header.frame_id = 'map'
        out.point.x, out.point.y, out.point.z = tx, ty, tz
        self.target_pub.publish(out)
        self.get_logger().info(f'검출 conf={conf:.2f} pixel=({cx:.0f},{cy:.0f}) depth={d:.2f}m -> NED=({tx:.2f},{ty:.2f},{tz:.2f})')


def main(args=None):
    rclpy.init(args=args)
    n = HailoYoloInferenceNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
