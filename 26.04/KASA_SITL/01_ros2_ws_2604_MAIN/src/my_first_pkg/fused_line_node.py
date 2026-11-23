#!/usr/bin/env python3
"""전방+하향 격자선 융합 측정 노드 (측정·로그 전용, setpoint 안 냄).

  python3 -u fused_line_node.py --ros-args -p csv:=/path/fused.csv
출력: CSV (EKF 위치 같이 기록 -> ulog 정렬·실제위치 채점용), /fused_line/debug 영상.
카메라: 2026-11-11 RPi GS(IMX296 1456x1088) — fx 파라미터로 렌즈 지정 (16mm 4637.7, 6mm 1739.1).
"""
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude

from fused_line import Cam, FusedLine, detect_segments

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
IMG_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
LIDAR_BELOW_BODY = 0.10   # model.sdf lidar z -0.1


def fmt(v):
    return "" if v is None else f"{v:.4f}"


class FusedLineNode(Node):
    def __init__(self):
        super().__init__("fused_line")
        self.declare_parameter("csv", "/tmp/fused_line.csv")
        self.declare_parameter("fx", 927.5)          # 하향 2026-11-14 3.2mm (4mm 1159.4) (2.8mm 811.6)
        self.declare_parameter("fx_front", 927.5)    # 전방 2026-11-14 3.2mm (4mm 1159.4) (44.6도 1775.0)
        self.declare_parameter("cx", 728.0)
        self.declare_parameter("cy", 544.0)
        self.declare_parameter("front_pitch", 0.6109)
        fx, cx, cy = (self.get_parameter(k).value for k in ("fx", "cx", "cy"))
        self.fl = FusedLine(Cam(fx, cx, cy, math.pi / 2, (0.0, 0.0, 0.15)),
                            Cam(self.get_parameter("fx_front").value, cx, cy, self.get_parameter("front_pitch").value, (0.10, 0.0, -0.02)))
        self.br = CvBridge()
        self.img = {"down": None, "front": None}
        self.lp = None
        self.q = None
        self.create_subscription(Image, "/camera/image_raw", lambda m: self.img.__setitem__("down", m), IMG_QOS)
        self.create_subscription(Image, "/camera_front/image_raw", lambda m: self.img.__setitem__("front", m), IMG_QOS)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
                                 lambda m: setattr(self, "lp", m), PX4_QOS)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude",
                                 lambda m: setattr(self, "q", list(m.q)), PX4_QOS)
        self.dbg = self.create_publisher(Image, "/fused_line/debug", 1)
        self.f = open(self.get_parameter("csv").value, "w")
        self.f.write("ros_t,px4_t,ekf_E,ekf_N,yaw,h,n_edge_d,n_edge_f,n_line_d,n_line_f,"
                     "lat_d,psi_d,lat_f,psi_f,lat_u,psi_u,cams_u,cross_d,cross_f,proc_ms\n")
        self.n = 0
        self.t_log = time.time()
        self.create_timer(0.05, self.tick)   # 20 Hz

    def tick(self):
        lp, q = self.lp, self.q
        if lp is None or q is None or not lp.dist_bottom_valid or any(v is None for v in self.img.values()):
            return
        t0 = time.time()
        h = lp.dist_bottom + LIDAR_BELOW_BODY
        yaw = lp.heading
        edges, frames = {}, {}
        for name in ("down", "front"):
            frames[name] = self.br.imgmsg_to_cv2(self.img[name], "bgr8")
            edges[name] = self.fl.edges(name, frames[name], q, h, yaw)
        al = {k: FusedLine.pair(edges[k], "along") for k in edges}
        cr = {k: FusedLine.pair(edges[k], "cross") for k in edges}
        fd = FusedLine.fit_along(al["down"])
        ff = FusedLine.fit_along(al["front"])
        fu = FusedLine.fit_along(al["down"] + al["front"])
        ms = (time.time() - t0) * 1000
        self.f.write(",".join([
            f"{time.time():.3f}", str(lp.timestamp), f"{lp.y:.3f}", f"{lp.x:.3f}", f"{yaw:.4f}", f"{h:.3f}",
            str(len(edges["down"])), str(len(edges["front"])), str(len(al["down"])), str(len(al["front"])),
            fmt(fd and fd[0]), fmt(fd and fd[1]), fmt(ff and ff[0]), fmt(ff and ff[1]),
            fmt(fu and fu[0]), fmt(fu and fu[1]), "+".join(sorted(fu[3])) if fu else "",
            " ".join(f"{x:.3f}" for x in FusedLine.crosses(cr["down"])),
            " ".join(f"{x:.3f}" for x in FusedLine.crosses(cr["front"])), f"{ms:.1f}"]) + "\n")
        self.n += 1
        if self.dbg.get_subscription_count() > 0:
            self.publish_debug(frames)
        if time.time() - self.t_log > 2.0:
            self.t_log = time.time()
            self.f.flush()
            self.get_logger().info(
                f"[fused] h={h:.2f} lat d/f/u={fmt(fd and fd[0])}/{fmt(ff and ff[0])}/{fmt(fu and fu[0])} "
                f"psi u={fmt(fu and fu[1])} lines d/f={len(al['down'])}/{len(al['front'])} {ms:.0f}ms n={self.n}")

    def publish_debug(self, frames):
        tiles = []
        for name in ("front", "down"):
            segs, g = detect_segments(frames[name], 0.5)
            v = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
            for x1, y1, x2, y2, dk, _ in segs:
                cv2.line(v, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255) if dk > 0 else (255, 0, 0), 2)
            cv2.putText(v, name, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 200, 0), 2)
            tiles.append(v)
        self.dbg.publish(self.br.cv2_to_imgmsg(np.hstack(tiles), "bgr8"))


def main():
    rclpy.init()
    n = FusedLineNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.f.close()


if __name__ == "__main__":
    main()
