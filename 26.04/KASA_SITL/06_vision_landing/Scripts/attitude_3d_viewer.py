#!/usr/bin/env python3
"""
PX4 자세각(Attitude) 실시간 3D 시각화
/fmu/out/vehicle_attitude 를 구독해서 쿼터니언으로 쿼드콥터 모델을 회전시켜 보여줍니다.

실행 전:
  source /opt/ros/foxy/setup.bash
  source ~/ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=0
  python3 ~/Desktop/attitude_3d_viewer.py

MicroXRCEAgent가 떠있어야 데이터가 들어옵니다:
  MicroXRCEAgent serial --dev /dev/ttyTHS0 -b 921600 &
"""
import threading
import time

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleAttitude

# ---- 쿼드콥터 와이어프레임 정의 (body FRD: X=앞, Y=오른쪽, Z=아래) ----
ARM = 1.0
body_lines = [
    # 십자 프레임 (앞-뒤, 좌-우 팔)
    ([ARM, 0, 0], [-ARM, 0, 0]),
    ([0, ARM, 0], [0, -ARM, 0]),
    # 노즈(기수) 표시 - 앞쪽으로 튀어나온 삼각형
    ([ARM, 0, 0], [ARM * 0.6, ARM * 0.25, 0]),
    ([ARM, 0, 0], [ARM * 0.6, -ARM * 0.25, 0]),
]
motor_points_body = np.array([
    [ARM, 0, 0],
    [-ARM, 0, 0],
    [0, ARM, 0],
    [0, -ARM, 0],
])


def quat_to_R(w, x, y, z):
    """PX4 q = [w,x,y,z], body(FRD) -> NED 회전행렬"""
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def ned_to_enu_display(p_ned):
    """NED(N,E,D) -> 화면표시용 (E,N,U) : X=동,Y=북,Z=위"""
    n, e, d = p_ned
    return np.array([e, n, -d])


class AttitudeNode(Node):
    def __init__(self, state):
        super().__init__("attitude_3d_viewer")
        self.state = state
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self.cb, qos
        )

    def cb(self, msg):
        self.state["q"] = (msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        self.state["last_update"] = time.time()
        self.state["count"] += 1


def quat_to_euler_deg(w, x, y, z):
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return 0.0, 0.0, 0.0
    w, x, y, z = w / n, x / n, y / n, z / n
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = np.arcsin(sinp)
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def main():
    state = {"q": (1.0, 0.0, 0.0, 0.0), "last_update": 0.0, "count": 0}

    rclpy.init()
    node = AttitudeNode(state)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig = plt.figure(figsize=(7, 7))
    fig.canvas.manager.set_window_title("PX4 Attitude 3D Viewer")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_zlim(-1.5, 1.5)
    ax.set_xlabel("East")
    ax.set_ylabel("North")
    ax.set_zlabel("Up")

    lc = Line3DCollection([], colors="k", linewidths=3)
    ax.add_collection3d(lc)
    nose_line = Line3DCollection([], colors="r", linewidths=4)
    ax.add_collection3d(nose_line)
    motor_scatter = ax.scatter([], [], [], color="b", s=60)

    status_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes, fontsize=10)

    def update(_frame):
        w, x, y, z = state["q"]
        R = quat_to_R(w, x, y, z)

        segs = []
        for p0, p1 in body_lines[:2]:
            a = ned_to_enu_display(R @ np.array(p0))
            b = ned_to_enu_display(R @ np.array(p1))
            segs.append([a, b])
        lc.set_segments(segs)

        nose_segs = []
        for p0, p1 in body_lines[2:]:
            a = ned_to_enu_display(R @ np.array(p0))
            b = ned_to_enu_display(R @ np.array(p1))
            nose_segs.append([a, b])
        nose_line.set_segments(nose_segs)

        pts = np.array([ned_to_enu_display(R @ p) for p in motor_points_body])
        motor_scatter._offsets3d = (pts[:, 0], pts[:, 1], pts[:, 2])

        age = time.time() - state["last_update"] if state["last_update"] else None
        roll, pitch, yaw = quat_to_euler_deg(w, x, y, z)
        if age is None:
            status = "데이터 없음 - Agent/토픽 확인 필요"
        elif age > 2.0:
            status = f"연결 끊김? (마지막 수신 {age:.1f}s 전)"
        else:
            status = f"Roll: {roll:6.1f}  Pitch: {pitch:6.1f}  Yaw: {yaw:6.1f}  (msgs: {state['count']})"
        status_text.set_text(status)

        return lc, nose_line, motor_scatter, status_text

    from matplotlib.animation import FuncAnimation
    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

    try:
        plt.show()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
