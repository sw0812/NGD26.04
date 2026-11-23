#!/usr/bin/env python3
"""ArUco-based visual position fix, published as VehicleOdometry on
/fmu/in/vehicle_visual_odometry so PX4's EKF2 can fuse it as a GPS
substitute (EKF2_EV_CTRL). Known marker world positions come from the
comments in kasa_indoor_search.world:
  vertiport (ID0): center of the world, top of the pad, z=0.7
  search waypoints: ID1(8,5) ID2(11,14) ID3(17,8) ID4(23,14), z=0 (floor)
Assumes each marker lies flat with its own +Z axis pointing straight up
(parallel to world Z) and no extra in-plane rotation - true for the
floor markers by construction; good enough as a first pass for ID0 too.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from scipy.spatial.transform import Rotation

from aruco_opencv_msgs.msg import ArucoDetection
from px4_msgs.msg import VehicleOdometry

MARKER_WORLD_POS = {
    0: (0.0, 0.0, 0.7),
    1: (8.0, 5.0, 0.0),
    2: (11.0, 14.0, 0.0),
    3: (17.0, 8.0, 0.0),
    4: (23.0, 14.0, 0.0),
}

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class VisionOdometryNode(Node):
    def __init__(self):
        super().__init__("vision_odometry_node")
        self.create_subscription(ArucoDetection, "/aruco_detections", self.on_detection, 10)
        self.pub = self.create_publisher(
            VehicleOdometry, "/fmu/in/vehicle_visual_odometry", PX4_QOS)
        self.count = 0
        self.get_logger().info("vision_odometry_node up, waiting for aruco detections")

    def on_detection(self, msg: ArucoDetection):
        if not msg.markers:
            return

        # use whichever known marker is detected; if several, average
        positions = []
        for m in msg.markers:
            if m.marker_id not in MARKER_WORLD_POS:
                continue
            p = m.pose.position
            q = m.pose.orientation
            t_cam_marker = [p.x, p.y, p.z]
            R_cam_marker = Rotation.from_quat([q.x, q.y, q.z, q.w])

            # camera position expressed in the marker's own local frame
            cam_in_marker = -R_cam_marker.inv().apply(t_cam_marker)

            mx, my, mz = MARKER_WORLD_POS[m.marker_id]
            # marker assumed world-axis-aligned (flat, facing up) -> world
            # position of the camera is just the marker's world position
            # plus that offset vector as-is.
            cam_world = [mx + cam_in_marker[0], my + cam_in_marker[1], mz + cam_in_marker[2]]
            positions.append(cam_world)

        if not positions:
            return

        avg = [sum(c[i] for c in positions) / len(positions) for i in range(3)]
        self.publish_odometry(avg)

    def publish_odometry(self, cam_world_xyz):
        x, y, z = cam_world_xyz
        msg = VehicleOdometry()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp_sample = msg.timestamp
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        # world (x east? no: we used plain x,y,z from the .world file, which
        # is a standard Gazebo ENU-ish world frame: x,y horizontal, z up).
        # PX4 NED wants north,east,down - treat world x as north, y as
        # east, and flip z for down, matching how we've been reading
        # LOCAL_POSITION_NED x/y directly against gazebo x/y all session.
        msg.position = [float(x), float(y), float(-z)]
        msg.q = [float("nan")] * 4  # orientation: let IMU/EKF keep its own
        msg.velocity = [float("nan")] * 3
        msg.angular_velocity = [float("nan")] * 3
        msg.position_variance = [0.05, 0.05, 0.05]
        msg.orientation_variance = [float("nan")] * 3
        msg.velocity_variance = [float("nan")] * 3
        msg.reset_counter = 0
        msg.quality = 1
        self.pub.publish(msg)
        self.count += 1
        if self.count % 10 == 1:
            self.get_logger().info(f"vision fix #{self.count}: x={x:.2f} y={y:.2f} z={z:.2f}")


def main():
    rclpy.init()
    node = VisionOdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
