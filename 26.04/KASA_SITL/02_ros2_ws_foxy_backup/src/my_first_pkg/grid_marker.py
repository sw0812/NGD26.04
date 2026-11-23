import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# 좌표를 이 파일에 직접 하드코딩하지 않고, kasa_gazebo_world 생성 스크립트가
# 만든 waypoints.json을 그대로 읽는다 — 규정 확정치가 바뀌면 이 파일도 손댈 것 없이
# 그 JSON만 다시 생성하면 됨 (Obsidian에 넣은 지도 이미지도 같은 파일 기준).
WAYPOINTS_JSON = (
    "/home/sasllab/Downloads/kasa_gazebo_world/models/kasa_ground_field/waypoints.json"
)


class GridMarker(Node):

    def __init__(self):
        super().__init__('grid_marker')

        with open(WAYPOINTS_JSON) as f:
            self.field = json.load(f)

        # RViz Fixed Frame과 반드시 일치해야 함. 이 프로젝트는 gz_pose_bridge.py가
        # 버티포트(출발점)=world 원점으로 TF를 쏘고 있어서 'world'로 맞춤 —
        # 'map'으로 두면 그 프레임을 쏘는 노드가 없어 RViz에 아무것도 안 뜬다.
        self.frame_id = 'world'

        # 정적인 지도 형상이라 매번 다시 안 그려도 되도록 TRANSIENT_LOCAL로 발행 —
        # RViz를 나중에 띄워도(늦게 구독해도) 마지막 상태를 바로 받는다.
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(MarkerArray, '/kasa_field_markers', qos)

        # 그래도 이 환경에서 ROS2 discovery가 가끔 불안정했으니(이전 세션들 참고),
        # 5초마다 한 번씩 재발행해서 안전망을 둠.
        self.timer = self.create_timer(5.0, self.publish_all)
        self.publish_all()

    def publish_all(self):
        arr = MarkerArray()
        arr.markers.append(self._grid_lines())
        arr.markers.append(self._grid_points())
        arr.markers.extend(self._grid_labels())
        arr.markers.append(self._boundary(self.field['mission_area'], "#8a7a4a", marker_id=10))
        arr.markers.append(self._boundary(self.field['safe_area'], "#9a9a9a", marker_id=11))
        arr.markers.append(self._vertiport())
        arr.markers.extend(self._aruco_waypoints())
        self.publisher.publish(arr)
        self.get_logger().info(f'KASA field markers published: {len(arr.markers)} markers')

    def _header(self, marker):
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        return marker

    def _grid_lines(self):
        m = self._header(Marker())
        m.ns, m.id = 'grid_lines', 0
        m.type, m.action = Marker.LINE_LIST, Marker.ADD
        m.scale.x = 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = 0.5, 0.5, 0.5, 1.0

        mis = self.field['mission_area']
        spacing = self.field['grid']['spacing']
        x0, x1 = mis['x_range']
        y0, y1 = mis['y_range']
        n_cols = int(round((x1 - x0) / spacing))
        n_rows = int(round((y1 - y0) / spacing))

        for i in range(n_cols + 1):
            x = x0 + i * spacing
            m.points += [Point(x=x, y=y0, z=0.01), Point(x=x, y=y1, z=0.01)]
        for j in range(n_rows + 1):
            y = y0 + j * spacing
            m.points += [Point(x=x0, y=y, z=0.01), Point(x=x1, y=y, z=0.01)]
        return m

    def _grid_points(self):
        m = self._header(Marker())
        m.ns, m.id = 'grid_points', 0
        m.type, m.action = Marker.POINTS, Marker.ADD
        m.scale.x = m.scale.y = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
        # 격자 경계가 아니라 "선이 만나는 안쪽 교차점"만 — ArUco 후보 지점과 같은 정의
        # (Obsidian 지도의 all_grid_intersections_28과 동일 데이터)
        for pt in self.field['all_grid_intersections_28']:
            m.points.append(Point(x=pt['x'], y=pt['y'], z=0.02))
        return m

    def _grid_labels(self):
        labels = []
        for i, pt in enumerate(self.field['all_grid_intersections_28']):
            m = self._header(Marker())
            m.ns, m.id = 'grid_labels', i
            m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = pt['x'], pt['y'] + 0.3, 0.05
            m.scale.z = 0.35
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.1, 0.1, 1.0
            m.text = f"({pt['x']:.0f},{pt['y']:.0f})"
            labels.append(m)
        return labels

    def _boundary(self, area, hex_color, marker_id):
        m = self._header(Marker())
        m.ns, m.id = 'boundary', marker_id
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.scale.x = 0.08
        r, g, b = (int(hex_color[i:i + 2], 16) / 255 for i in (1, 3, 5))
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        x0, x1 = area['x_range']
        y0, y1 = area['y_range']
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        m.points = [Point(x=x, y=y, z=0.0) for x, y in corners]
        return m

    def _vertiport(self):
        vp = self.field['vertiport']
        m = self._header(Marker())
        m.ns, m.id = 'vertiport', 0
        m.type, m.action = Marker.CYLINDER, Marker.ADD
        m.pose.position.x, m.pose.position.y = vp['x'], vp['y']
        m.pose.position.z = vp['height_m'] / 2
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = vp['diameter_m']
        m.scale.z = vp['height_m']
        m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.9, 0.9, 0.9
        return m

    def _aruco_waypoints(self):
        markers = []
        for wp in self.field['scored_waypoints_aruco']:
            cube = self._header(Marker())
            cube.ns, cube.id = 'aruco_waypoints', wp['id']
            cube.type, cube.action = Marker.CUBE, Marker.ADD
            cube.pose.position.x, cube.pose.position.y = wp['x'], wp['y']
            cube.pose.position.z = 0.05
            cube.pose.orientation.w = 1.0
            cube.scale.x = cube.scale.y = wp['marker_size_m']
            cube.scale.z = 0.02
            cube.color.r, cube.color.g, cube.color.b, cube.color.a = 0.05, 0.05, 0.05, 1.0
            markers.append(cube)

            label = self._header(Marker())
            label.ns, label.id = 'aruco_labels', wp['id']
            label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x, label.pose.position.y = wp['x'], wp['y']
            label.pose.position.z = 0.6
            label.scale.z = 0.45
            label.color.r, label.color.g, label.color.b, label.color.a = 0.7, 0.25, 0.0, 1.0
            label.text = f"ID{wp['id']}"
            markers.append(label)
        return markers


def main(args=None):
    rclpy.init(args=args)
    node = GridMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
