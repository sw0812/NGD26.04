#!/usr/bin/env python3
"""EKF 데이터 수집용 긴 호버 노드 (2026-11-13, 측정 전용 비행).

이륙 -> 지정 지점(world x=E, y=N)으로 이동 -> hold_s 동안 위치 고정 호버 -> AUTO.LAND -> disarm 후 종료.
  python3 -u hover_node.py --ros-args -p e:=5.0 -p n:=5.0 -p alt:=2.0 -p hold_s:=180.0
PX4 위치 setpoint(오프보드). 고도는 home(버티포트 상판) 기준 alt [m].
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)


class Hover(Node):
    def __init__(self):
        super().__init__("hover_node")
        from rcl_interfaces.msg import ParameterDescriptor
        for k, v in (("e", 5.0), ("n", 5.0), ("alt", 2.0), ("hold_s", 180.0), ("speed", 1.0)):
            # 정수(180)로 넘겨도 받도록 dynamic_typing (2026-11-13 밤샘 첫 호버가 INTEGER 타입 오류로 죽음)
            self.declare_parameter(k, v, ParameterDescriptor(dynamic_typing=True))
        g = lambda k: float(self.get_parameter(k).value)
        self.tgt_e, self.tgt_n, self.alt, self.hold_s, self.speed = g("e"), g("n"), g("alt"), g("hold_s"), g("speed")
        self.lp = None
        self.status = None
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
                                 lambda m: setattr(self, "lp", m), QOS)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4",
                                 lambda m: setattr(self, "status", m), QOS)
        self.ocm = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", QOS)
        self.sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", QOS)
        self.cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", QOS)
        self.phase = "ARM"
        self.ticks = 0
        self.t_phase = self.now()
        self.cur = None   # 이동 중 setpoint (n, e)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f"[HOVER] 목표 E{self.tgt_e} N{self.tgt_n} 고도 {self.alt} m, {self.hold_s:.0f} s")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def command(self, c, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = int(self.now() * 1e6)
        m.command, m.param1, m.param2 = c, float(p1), float(p2)
        m.target_system = m.target_component = m.source_system = m.source_component = 1
        m.from_external = True
        self.cmd.publish(m)

    def set_phase(self, p):
        self.get_logger().info(f"[HOVER] {self.phase} -> {p}")
        self.phase, self.t_phase = p, self.now()

    def tick(self):
        self.ticks += 1
        if self.lp is None or self.status is None:
            return
        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if self.phase == "LAND":
            if not armed and self.now() - self.t_phase > 3.0:
                self.get_logger().info("[HOVER] disarm 확인 -> 종료")
                raise SystemExit(0)
            return
        o = OffboardControlMode()
        o.position = True
        o.timestamp = int(self.now() * 1e6)
        self.ocm.publish(o)
        n, e, z = self.lp.x, self.lp.y, self.lp.z
        if self.cur is None:
            self.cur = (n, e)
            # 이륙 시 기수 유지 (2026-11-13: yaw=0 고정이라 GPS끔에서 90° 회전 -> 헤딩 20~30° 틀어져 한 칸 밀림)
            self.yaw0 = float(self.lp.heading)
        tz = -self.alt
        if self.phase == "ARM":
            if self.ticks > 20 and self.ticks % 20 == 0:
                if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                if not armed:
                    self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if armed and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.set_phase("CLIMB")
        elif self.phase == "CLIMB":
            if abs(z - tz) < 0.2:
                self.set_phase("GOTO")
        elif self.phase == "GOTO":
            dn, de = self.tgt_n - self.cur[0], self.tgt_e - self.cur[1]
            d = math.hypot(dn, de)
            step = self.speed * 0.05
            self.cur = (self.tgt_n, self.tgt_e) if d < step else (self.cur[0] + dn / d * step, self.cur[1] + de / d * step)
            if d < step and math.hypot(self.tgt_n - n, self.tgt_e - e) < 0.3:
                self.set_phase("HOLD")
        elif self.phase == "HOLD":
            if self.now() - self.t_phase > self.hold_s:
                self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.set_phase("LAND")
                return
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.now() * 1e6)
        sp.position = [float(self.cur[0]), float(self.cur[1]), float(tz)]
        sp.velocity = [float("nan")] * 3
        sp.yaw = self.yaw0
        self.sp.publish(sp)


def main():
    rclpy.init()
    try:
        rclpy.spin(Hover())
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == "__main__":
    main()
