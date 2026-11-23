"""ROS 이미지 토픽 한 장씩 PNG로 저장: python3 snap_topics.py <out_dir> /topic1 /topic2 ..."""
import sys, time, rclpy, cv2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
out = sys.argv[1]; topics = sys.argv[2:]
rclpy.init(); n = Node('snap'); b = CvBridge(); got = {}
def cb(t):
    def f(m):
        if t not in got:
            got[t] = 1
            cv2.imwrite(f"{out}/{t.strip('/').replace('/', '_')}.png", b.imgmsg_to_cv2(m, 'bgr8'))
            print(t, m.width, m.height)
    return f
for t in topics: n.create_subscription(Image, t, cb(t), qos_profile_sensor_data)
t0 = time.time()
while len(got) < len(topics) and time.time() - t0 < 15: rclpy.spin_once(n, timeout_sec=0.2)
