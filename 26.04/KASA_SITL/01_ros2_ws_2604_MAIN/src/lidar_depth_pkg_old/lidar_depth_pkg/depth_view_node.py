#!/usr/bin/env python3
"""
/depth/image (32FC1) -> 정규화된 8bit grayscale 이미지로 변환해서
/depth/image_view (mono8) 로 발행하는 시각화용 노드.

rqt_image_view 로 /depth/image_view 를 구독하면 육안으로 depth map을
확인할 수 있다. 원본 depth map은 세로가 16픽셀(라이다 vertical
samples)밖에 안 돼서 그대로 보면 너무 납작하니, vertical_upscale
배수만큼 각 행을 반복해서 보기 편하게 늘린다.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class DepthViewNode(Node):
    def __init__(self):
        super().__init__('depth_view_node')

        self.declare_parameter('input_topic', '/depth/image')
        self.declare_parameter('output_topic', '/depth/image_view')
        self.declare_parameter('vertical_upscale', 5)
        self.declare_parameter('render_min', 0.5)
        self.declare_parameter('render_max', 12.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.upscale = int(self.get_parameter('vertical_upscale').value)
        self.render_min = float(self.get_parameter('render_min').value)
        self.render_max = float(self.get_parameter('render_max').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(
            Image, self.input_topic, self.image_callback, qos
        )
        self.pub = self.create_publisher(Image, self.output_topic, 10)

        self.get_logger().info(
            f'DepthViewNode 시작: {self.input_topic} -> {self.output_topic}'
        )

    def image_callback(self, msg: Image):
        if msg.encoding != '32FC1':
            self.get_logger().warn(f'예상치 못한 encoding: {msg.encoding}')
            return

        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width
        )

        valid = (depth >= self.render_min) & (depth <= self.render_max)
        norm = np.zeros_like(depth, dtype=np.float32)
        span = max(self.render_max - self.render_min, 1e-6)
        # 가까울수록 밝게 반전 (render_min -> 255, render_max -> 0)
        norm[valid] = 1.0 - (depth[valid] - self.render_min) / span
        gray = (np.clip(norm, 0.0, 1.0) * 255).astype(np.uint8)

        if self.upscale > 1:
            gray = np.repeat(gray, self.upscale, axis=0)

        out_msg = Image()
        out_msg.header = msg.header
        out_msg.height, out_msg.width = gray.shape
        out_msg.encoding = 'mono8'
        out_msg.is_bigendian = 0
        out_msg.step = out_msg.width
        out_msg.data = gray.tobytes()
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthViewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
