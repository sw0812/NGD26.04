#!/usr/bin/env python3
"""
A* 기반 경로계획 노드.

/depth/image(LiDAR 극좌표 격자)로 주변 장애물을 파악해 로컬 2D occupancy grid를
만들고, 드론 현재 위치 -> 착륙 타겟까지 A*로 최단 경로를 탐색해서
/planned_path (nav_msgs/Path, local NED)로 발행한다.

APF(인공 포텐셜 필드) 방식의 구조적 한계였던
  - local minima (인력/척력 상쇄로 정지)
  - 좁은 통로 진동 / 통과 불가
  - 목표 옆 장애물로 인한 도달 실패
를 전역 탐색으로 해결한다.

시각화: /planned_path_markers (visualization_msgs/MarkerArray)로 RViz에서 확인 가능.
"""

import heapq
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from px4_msgs.msg import VehicleLocalPosition

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        p = self.declare_parameter
        p('depth_topic', '/depth/image')
        p('target_topic', '/landing_target')
        p('local_pos_topic', '/fmu/out/vehicle_local_position_v1')
        p('path_topic', '/planned_path')
        p('marker_topic', '/planned_path_markers')
        # 격자 설정 (드론 중심 로컬 맵)
        p('grid_size', 40)          # 40x40 셀
        p('cell_size', 0.25)        # 셀당 0.25m -> 10m x 10m 영역
        p('inflate_radius', 0.5)    # 장애물 팽창(드론 반경 + 여유), m
        p('plan_rate', 2.0)         # 재계획 주기(Hz)
        # 장애물 판정 (depth 격자에서)
        p('h_samples', 360)
        p('v_samples', 64)
        p('v_min_deg', -50.0)
        p('v_max_deg', 50.0)
        p('obstacle_min_range', 0.3)
        p('obstacle_max_range', 6.0)
        p('safe_horizontal_radius', 0.6)  # 이 수평거리 이내는 패드/지면으로 간주해 제외

        g = lambda n: self.get_parameter(n).value
        self.grid_size = int(g('grid_size'))
        self.cell_size = float(g('cell_size'))
        self.inflate_radius = float(g('inflate_radius'))
        self.h_samples = int(g('h_samples'))
        self.v_samples = int(g('v_samples'))
        self.v_min = math.radians(float(g('v_min_deg')))
        self.v_max = math.radians(float(g('v_max_deg')))
        self.obstacle_min_range = float(g('obstacle_min_range'))
        self.obstacle_max_range = float(g('obstacle_max_range'))
        self.safe_horizontal_radius = float(g('safe_horizontal_radius'))

        # LiDAR 격자 방향코사인 사전계산
        rows = np.arange(self.v_samples)
        cols = np.arange(self.h_samples)
        elev = self.v_min + (rows + 0.5) / self.v_samples * (self.v_max - self.v_min)
        az = (cols + 0.5) / self.h_samples * (2 * math.pi) - math.pi
        eg, ag = np.meshgrid(elev, az, indexing='ij')
        self.dir_x = np.cos(eg) * np.cos(ag)
        self.dir_y = np.cos(eg) * np.sin(ag)

        qos_img = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Image, g('depth_topic'), self.depth_cb, qos_img)
        self.create_subscription(PointStamped, g('target_topic'), self.target_cb, 10)
        self.create_subscription(VehicleLocalPosition, g('local_pos_topic'),
                                 self.local_pos_cb, PX4_QOS)

        self.path_pub = self.create_publisher(Path, g('path_topic'), 10)
        self.marker_pub = self.create_publisher(MarkerArray, g('marker_topic'), 10)

        self.drone_x = self.drone_y = self.drone_z = 0.0
        self.have_pos = False
        self.target_x = self.target_y = self.target_z = None
        self.obstacles_body = []  # [(dx, dy)] 드론 기준 상대 좌표

        self.timer = self.create_timer(1.0 / float(g('plan_rate')), self.plan_cb)
        self.get_logger().info(
            f'PathPlannerNode 시작: {self.grid_size}x{self.grid_size} grid, '
            f'cell={self.cell_size}m ({self.grid_size*self.cell_size:.1f}m 영역)'
        )

    # ---------- 콜백 ----------
    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.drone_x, self.drone_y, self.drone_z = msg.x, msg.y, msg.z
        self.have_pos = True

    def target_cb(self, msg: PointStamped):
        self.target_x, self.target_y, self.target_z = \
            msg.point.x, msg.point.y, msg.point.z

    def depth_cb(self, msg: Image):
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width)
        px = self.dir_x * depth
        py = self.dir_y * depth
        horiz = np.sqrt(px ** 2 + py ** 2)

        valid = (
            (depth >= self.obstacle_min_range)
            & (horiz > self.safe_horizontal_radius)
            & (horiz <= self.obstacle_max_range)
        )
        if not np.any(valid):
            self.obstacles_body = []
            return

        # lidar(ENU-body) -> NED body: y 부호 반전
        ox = px[valid]
        oy = -py[valid]
        # 셀 단위로 중복 제거해서 개수 줄임
        keys = set()
        pts = []
        for x, y in zip(ox, oy):
            k = (int(round(x / self.cell_size)), int(round(y / self.cell_size)))
            if k not in keys:
                keys.add(k)
                pts.append((float(x), float(y)))
        self.obstacles_body = pts

    # ---------- 격자/A* ----------
    def world_to_cell(self, x, y):
        """드론 기준 상대좌표(m) -> 격자 인덱스. 격자 중심이 드론."""
        half = self.grid_size // 2
        cx = int(round(x / self.cell_size)) + half
        cy = int(round(y / self.cell_size)) + half
        return cx, cy

    def cell_to_world(self, cx, cy):
        half = self.grid_size // 2
        return (cx - half) * self.cell_size, (cy - half) * self.cell_size

    def build_grid(self):
        n = self.grid_size
        grid = np.zeros((n, n), dtype=bool)  # True = 장애물
        inflate_cells = int(math.ceil(self.inflate_radius / self.cell_size))

        for ox, oy in self.obstacles_body:
            cx, cy = self.world_to_cell(ox, oy)
            for dx in range(-inflate_cells, inflate_cells + 1):
                for dy in range(-inflate_cells, inflate_cells + 1):
                    if dx * dx + dy * dy > inflate_cells * inflate_cells:
                        continue
                    x, y = cx + dx, cy + dy
                    if 0 <= x < n and 0 <= y < n:
                        grid[x, y] = True
        return grid

    def astar(self, grid, start, goal):
        n = self.grid_size
        if not (0 <= goal[0] < n and 0 <= goal[1] < n):
            return None
        # 목표가 팽창 영역에 걸리면 목표 셀 주변은 통행 허용(착륙 지점이므로)
        grid = grid.copy()
        grid[goal[0], goal[1]] = False

        def h(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1),
                     (-1, -1), (-1, 1), (1, -1), (1, 1)]

        open_set = [(h(start, goal), 0.0, start, None)]
        came = {}
        gscore = {start: 0.0}
        closed = set()

        while open_set:
            _, g, cur, parent = heapq.heappop(open_set)
            if cur in closed:
                continue
            closed.add(cur)
            came[cur] = parent
            if cur == goal:
                path = []
                node = cur
                while node is not None:
                    path.append(node)
                    node = came[node]
                return list(reversed(path))

            for dx, dy in neighbors:
                nx, ny = cur[0] + dx, cur[1] + dy
                if not (0 <= nx < n and 0 <= ny < n):
                    continue
                if grid[nx, ny]:
                    continue
                step = math.hypot(dx, dy)
                ng = g + step
                if ng < gscore.get((nx, ny), float('inf')):
                    gscore[(nx, ny)] = ng
                    heapq.heappush(open_set, (ng + h((nx, ny), goal), ng, (nx, ny), cur))
        return None

    def simplify_path(self, cells):
        """직선으로 이어지는 연속 셀은 중간점 제거(웨이포인트 수 축소)."""
        if len(cells) <= 2:
            return cells
        out = [cells[0]]
        for i in range(1, len(cells) - 1):
            px, py = cells[i - 1]
            cx, cy = cells[i]
            nx, ny = cells[i + 1]
            if (cx - px, cy - py) != (nx - cx, ny - cy):
                out.append(cells[i])
        out.append(cells[-1])
        return out

    # ---------- 메인 ----------
    def plan_cb(self):
        if not self.have_pos or self.target_x is None:
            return

        # 목표를 드론 기준 상대좌표로
        rel_x = self.target_x - self.drone_x
        rel_y = self.target_y - self.drone_y

        grid = self.build_grid()
        half = self.grid_size // 2
        start = (half, half)
        goal = self.world_to_cell(rel_x, rel_y)

        # 목표가 격자 밖이면 격자 경계로 클램프(그 방향으로 일단 이동)
        goal = (max(0, min(self.grid_size - 1, goal[0])),
                max(0, min(self.grid_size - 1, goal[1])))

        cells = self.astar(grid, start, goal)
        if cells is None:
            self.get_logger().warn('경로 없음 - 장애물에 완전히 막힘', throttle_duration_sec=2.0)
            return

        cells = self.simplify_path(cells)

        # Path 메시지 (절대 NED 좌표)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for cx, cy in cells:
            wx, wy = self.cell_to_world(cx, cy)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = self.drone_x + wx
            ps.pose.position.y = self.drone_y + wy
            ps.pose.position.z = self.drone_z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)
        self.publish_markers(path, grid)

    def publish_markers(self, path: Path, grid):
        arr = MarkerArray()

        line = Marker()
        line.header = path.header
        line.ns = 'planned_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.08
        line.color.r, line.color.g, line.color.b, line.color.a = 0.0, 1.0, 0.0, 1.0
        for ps in path.poses:
            line.points.append(ps.pose.position)
        arr.markers.append(line)

        obs = Marker()
        obs.header = path.header
        obs.ns = 'obstacles'
        obs.id = 1
        obs.type = Marker.CUBE_LIST
        obs.action = Marker.ADD
        obs.scale.x = obs.scale.y = self.cell_size
        obs.scale.z = 0.1
        obs.color.r, obs.color.g, obs.color.b, obs.color.a = 1.0, 0.0, 0.0, 0.6
        for ox, oy in self.obstacles_body:
            from geometry_msgs.msg import Point
            pt = Point()
            pt.x = self.drone_x + ox
            pt.y = self.drone_y + oy
            pt.z = self.drone_z
            obs.points.append(pt)
        arr.markers.append(obs)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
