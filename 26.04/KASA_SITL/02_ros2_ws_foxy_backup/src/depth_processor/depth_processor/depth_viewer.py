import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np


class DepthViewer(Node):

    def __init__(self):
        super().__init__("depth_viewer")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            "/depth/image",
            self.image_callback,
            10,
        )

    def image_callback(self, msg):

        depth = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="passthrough"
        )

        h, w = depth.shape

        center_x = w // 2
        center_y = h // 2

        center_depth = depth[center_y, center_x]

        self.get_logger().info(
            f"Center Depth : {center_depth:.3f} m"
        )


def main(args=None):

    rclpy.init(args=args)

    node = DepthViewer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
