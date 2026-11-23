#!/usr/bin/env python3
"""
카메라에 구획선(라인)이 "보이는 동안 계속" 모터를 돌리고, 안 보이면 멈추는
스크립트. marker_motor_trigger.py의 라인트레이서 버전.

MAV_CMD_ACTUATOR_TEST(310)를 씀 - disarm 상태에서만 동작하고 위치추정이
전혀 필요 없음. 검출은 main.py의 "라인트레이서" 카메라 모드와 동일한
line_tracker.py(LineTracker)를 그대로 재사용(로직 복제 안 함).

usb_cam이 켜져 있어야 함(/camera/image_raw, rgb8 인코딩 전제 -
main.py SUBSYSTEMS["camera"]와 동일 설정으로 실행할 것).

사용법 (프로펠러 반드시 제거 후):
  python3 line_motor_trigger.py --motor 1 --value 0.15
"""
import argparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image as RosImage
from px4_msgs.msg import VehicleCommand

# line_tracker.py가 이 파일과 같은 디렉터리에 있어 sys.path 조작 없이도 바로 import됨
from line_tracker import LineTracker

MAV_CMD_ACTUATOR_TEST = 310
REFRESH_TIMEOUT_SEC = 0.4  # 이 시간 안에 새 명령이 없으면 PX4가 자동 정지


class LineMotorTrigger(Node):
    def __init__(self, args):
        super().__init__("line_motor_trigger")
        self.args = args
        self.was_visible = False
        self.line_tracker = LineTracker()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)
        self.currently_visible = False
        self.create_subscription(RosImage, "/camera/image_raw", self._on_image, 5)
        # 카메라 콜백(수십 Hz)마다 바로 발행하면 uXRCE-DDS 시리얼 채널이 밀려서
        # 명령이 아예 안 가는 문제가 marker_motor_trigger.py에서 실측됨
        # (2026-09-22) - 여기도 동일하게 발행은 10Hz 타이머에서만 하고,
        # 이미지 콜백은 "지금 보이는지" 상태만 갱신하도록 분리.
        self.create_timer(0.1, self._on_timer)
        self.get_logger().info(
            f"[line_motor_trigger] 라인이 보이는 동안 계속 모터 {args.motor}번을 "
            f"value={args.value}로 회전, 안 보이면 {REFRESH_TIMEOUT_SEC}초 이내 자동 정지. "
            "프로펠러 반드시 제거된 상태에서만 사용할 것."
        )

    def _on_image(self, msg: RosImage):
        if msg.encoding != "rgb8":
            return
        arr_rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        bgr = cv2.cvtColor(arr_rgb, cv2.COLOR_RGB2BGR)
        result = self.line_tracker.process(bgr)
        self.currently_visible = result.line_found

    def _send_actuator_test(self, value, timeout):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = MAV_CMD_ACTUATOR_TEST
        msg.param1 = float(value)
        msg.param2 = float(timeout)
        msg.param5 = float(self.args.motor)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def _on_timer(self):
        visible = self.currently_visible
        if visible:
            # 10Hz로만 갱신 발사 - 라인이 계속 보이는 한 정지 타임아웃이 계속 미뤄짐
            self._send_actuator_test(self.args.value, REFRESH_TIMEOUT_SEC)
            if not self.was_visible:
                self.get_logger().info("라인 인식됨 -> 회전 시작")
        elif self.was_visible:
            # 즉시 정지 명령도 같이 보내서 최대 대기시간(0.4s) 없이 바로 멈추게 함
            self._send_actuator_test(float("nan"), 0.0)
            self.get_logger().info("라인 사라짐 -> 정지")
        self.was_visible = visible


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--motor", type=int, default=1)
    ap.add_argument("--value", type=float, default=0.15)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = LineMotorTrigger(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 확실히 정지
        node._send_actuator_test(float("nan"), 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
