#!/usr/bin/env python3
"""KASA 미션 제어 GUI.

규정 반영:
  - "개수는 사전 공지... 참가팀은 비행 전 그 개수를 시스템에 입력"
    -> 마커 개수 입력창 (search_phase_node.py의 n_markers_expected 파라미터로 전달)
  - "임무 시작은... GCS에서 '임무 시작' 명령 1회만 사람이 트리거. 그 이후 전
    과정은 기체가 완전 자율 수행."
    -> '미션 시작' 버튼 1개만 누르면 그 뒤로는 search_phase_node.py가 알아서 함

실시간 상태(위치/목표/ARM/OFFBOARD)는 search_phase_node.py가 이미 발행 중인
PX4 토픽 + /kasa_search_markers를 그대로 구독해서 보여줌 — 새 통신 경로를
따로 안 만들고 기존 노드가 쏘는 정보를 재사용.
"""
import os
import queue
import subprocess
import sys
import threading
import tkinter as tk
from datetime import datetime
from tkinter import filedialog, messagebox, ttk
from tkinter.scrolledtext import ScrolledText

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from visualization_msgs.msg import MarkerArray

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

NODE_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "search_phase_node.py")
STATUS_POLL_MS = 200

# [[KASA 시연 실행 순서]] 문서의 터미널 1~7을 그대로 옮김 (터미널 8=search_phase_node.py는
# 아래 '미션 시작' 버튼이 파라미터까지 붙여서 대신함). 순서·대기(예: SITL의
# "Ready for takeoff" 뜬 뒤 2번) 판단은 사람이 직접 하도록 버튼을 자동 연쇄시키지
# 않고 하나씩 누르게 둠.
WORLD_NORMAL = "/home/sasllab/Downloads/kasa_gazebo_world/worlds/kasa_indoor_search.world"
WORLD_WINDY = "/home/sasllab/Downloads/kasa_gazebo_world/worlds/kasa_indoor_search_windy.world"


def sitl_commands(world_path):
    """1번 스텝(SITL) 커맨드 — 바람 켜기 체크박스에 따라 월드 파일만 바꿔서
    씀. Gazebo는 월드를 로드할 때만 플러그인 설정을 읽어서, 바람 세기를
    실행 중에 못 바꾸고 이 스텝을 다시 켜야 반영됨(끄려면 그냥 체크 해제
    하고 1번 다시 실행)."""
    return [
        "source /opt/ros/foxy/setup.bash",
        "cd ~/PX4-Autopilot",
        "export GAZEBO_MODEL_PATH=/home/sasllab/Downloads/kasa_gazebo_world/models:$GAZEBO_MODEL_PATH",
        f"export PX4_SITL_WORLD={world_path}",
        "./Tools/simulation/gazebo-classic/sitl_run.sh $(pwd)/build/px4_sitl_default/bin/px4 "
        "none iris_aruco_cam none $(pwd) $(pwd)/build/px4_sitl_default",
    ]


TERMINAL_STEPS = [
    ("2. Gazebo GUI (Ready for takeoff 이후)", [
        "~/Downloads/kasa_gazebo_world/scripts/run_gzclient.sh",
    ]),
    ("3. gz_pose_bridge", [
        "source /opt/ros/foxy/setup.bash",
        "export GAZEBO_MODEL_PATH=/home/sasllab/Downloads/kasa_gazebo_world/models:"
        "/home/sasllab/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models",
        "python3 ~/ros2_ws/gz_pose_bridge.py",
    ]),
    ("4. grid_marker", [
        "source /opt/ros/foxy/setup.bash",
        "python3 ~/ros2_ws/src/my_first_pkg/grid_marker.py",
    ]),
    ("5. RViz2", [
        "source /opt/ros/foxy/setup.bash",
        "rviz2 -d ~/ros2_ws/kasa_view.rviz",
    ]),
    ("6. ArUco tracker", [
        "source /opt/ros/foxy/setup.bash",
        "ros2 run aruco_opencv aruco_tracker_autostart --ros-args "
        "--params-file ~/PX4-Autopilot/aruco_tracker.yaml",
    ]),
    ("7. MicroXRCEAgent", [
        "MicroXRCEAgent udp4 -p 8888",
    ]),
]


def launch_terminal(commands):
    """새 터미널 창을 띄워 commands를 순서대로 실행. 끝나거나 죽어도 창은
    안 닫히게(exec bash) 해서 로그를 읽거나 필요하면 그 안에서 Ctrl+C 가능."""
    script = " && ".join(commands) + "; exec bash"
    subprocess.Popen(["gnome-terminal", "--", "bash", "-c", script], env=os.environ.copy())


class TelemetryNode(Node):
    """백그라운드 스레드에서 spin. GUI 쪽은 이 인스턴스의 속성만 주기적으로 읽는다."""

    def __init__(self):
        super().__init__("kasa_gui_telemetry")
        self.world_x = 0.0
        self.world_y = 0.0
        self.armed = False
        self.offboard = False
        self.state_text = "—"
        self.target_xy = None
        self.local_position_received = False

        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
            self._on_local_pos, PX4_QOS)
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v4", self._on_status, PX4_QOS)
        self.create_subscription(
            MarkerArray, "/kasa_search_markers", self._on_markers, 10)

    def _on_local_pos(self, msg):
        # PX4 local(NED, x=North,y=East) -> world(ENU) — search_phase_node.py와 동일 변환
        self.world_x = msg.y
        self.world_y = msg.x
        self.local_position_received = True

    def _on_status(self, msg):
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def _on_markers(self, msg):
        for m in msg.markers:
            if m.ns == "search_state":
                self.state_text = m.text
            elif m.ns == "search_current_cell":
                self.target_xy = (m.pose.position.x, m.pose.position.y)


class MissionGUI:
    def __init__(self, root, telemetry):
        self.root = root
        self.telemetry = telemetry
        self.proc = None
        self.log_queue = queue.Queue()

        root.title("KASA 미션 제어")
        root.geometry("480x760")
        root.minsize(420, 600)

        frm = ttk.Frame(root, padding=16)
        frm.pack(fill="both", expand=True)

        ttk.Label(frm, text="시뮬레이션 스택 (순서대로 하나씩)", font=("", 11)).pack(anchor="w")
        ttk.Label(
            frm, text="※ 1번 'Ready for takeoff' 뜨는 거 보고 2번부터 눌러야 함",
            font=("", 8), foreground="gray", wraplength=300, justify="left",
        ).pack(anchor="w")
        step_frm = ttk.Frame(frm)
        step_frm.pack(fill="x", pady=(4, 12))

        step1_row = ttk.Frame(step_frm)
        step1_row.pack(fill="x", pady=1)
        self.wind_var = tk.BooleanVar(value=False)
        ttk.Button(
            step1_row, text="1. SITL (PX4 + Gazebo)",
            command=lambda: launch_terminal(sitl_commands(
                WORLD_WINDY if self.wind_var.get() else WORLD_NORMAL)),
        ).pack(side="left", fill="x", expand=True)
        ttk.Checkbutton(
            step1_row, text="바람", variable=self.wind_var, width=6,
        ).pack(side="left", padx=(4, 0))

        for label, commands in TERMINAL_STEPS:
            ttk.Button(
                step_frm, text=label,
                command=lambda c=commands: launch_terminal(c),
            ).pack(fill="x", pady=1)

        ttk.Separator(frm).pack(fill="x", pady=(0, 12))

        ttk.Label(frm, text="경로점(마커) 개수", font=("", 11)).pack(anchor="w")
        ttk.Label(
            frm, text="※ 대회 당일 통보된 값을 입력 (규정: 사전 미확정)",
            font=("", 8), foreground="gray",
        ).pack(anchor="w")
        self.marker_count_var = tk.StringVar(value="4")
        self.entry = ttk.Entry(frm, textvariable=self.marker_count_var, width=8)
        self.entry.pack(anchor="w", pady=(2, 12))

        btn_row = ttk.Frame(frm)
        btn_row.pack(fill="x")
        self.start_btn = ttk.Button(btn_row, text="미션 시작", command=self.start_mission)
        self.start_btn.pack(side="left", expand=True, fill="x", padx=(0, 4))
        self.stop_btn = ttk.Button(
            btn_row, text="중지", command=self.stop_mission, state="disabled")
        self.stop_btn.pack(side="left", expand=True, fill="x", padx=(4, 0))

        ttk.Separator(frm).pack(fill="x", pady=12)

        self.state_label = self._status_row(frm, "상태")
        self.pos_label = self._status_row(frm, "현재 위치")
        self.target_label = self._status_row(frm, "목표 좌표")
        self.arm_label = self._status_row(frm, "ARM")
        self.offboard_label = self._status_row(frm, "OFFBOARD")

        ttk.Separator(frm).pack(fill="x", pady=12)

        log_header = ttk.Frame(frm)
        log_header.pack(fill="x")
        ttk.Label(log_header, text="실행 로그 (search_phase_node.py)", font=("", 11)).pack(
            side="left")
        self.save_log_btn = ttk.Button(
            log_header, text="로그 저장", command=self.save_log)
        self.save_log_btn.pack(side="right")
        ttk.Label(
            frm, text="※ 미션 시작 눌렀는데 안 되면 여기서 원인 확인",
            font=("", 8), foreground="gray",
        ).pack(anchor="w", pady=(0, 4))
        self.log_text = ScrolledText(
            frm, height=12, font=("Monospace", 9),
            background="#1b2330", foreground="#e7ece8", insertbackground="#e7ece8",
            wrap="word", state="disabled",
        )
        self.log_text.pack(fill="both", expand=True)
        self.log_text.tag_configure("error", foreground="#ff8a75")

        self.update_status()

    def save_log(self):
        content = self.log_text.get("1.0", "end")
        default_name = datetime.now().strftime("kasa_log_%Y%m%d_%H%M%S.txt")
        path = filedialog.asksaveasfilename(
            title="로그 저장", initialdir=os.path.expanduser("~/Desktop"),
            initialfile=default_name, defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
        )
        if not path:
            return
        with open(path, "w") as f:
            f.write(content)
        messagebox.showinfo("저장 완료", f"로그 저장됨:\n{path}")

    def _append_log(self, line):
        tag = "error" if any(k in line for k in ("ERROR", "Traceback", "Error:")) else None
        self.log_text.config(state="normal")
        self.log_text.insert("end", line, tag)
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def _status_row(self, parent, label_text):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=3)
        ttk.Label(row, text=f"{label_text}:", width=10).pack(side="left")
        value = ttk.Label(row, text="—", font=("", 10, "bold"))
        value.pack(side="left")
        return value

    # ------------------------------------------------------------------

    def start_mission(self):
        try:
            n = int(self.marker_count_var.get())
            if n <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("입력 오류", "마커 개수는 1 이상의 정수여야 합니다.")
            return

        if self.proc is not None and self.proc.poll() is None:
            messagebox.showwarning("이미 실행 중", "미션이 이미 진행 중입니다.")
            return

        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")

        self.proc = subprocess.Popen(
            [sys.executable, NODE_SCRIPT,
             "--ros-args", "-p", f"n_markers_expected:={n}"],
            env=os.environ.copy(),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1,
        )
        threading.Thread(
            target=self._pump_proc_output, args=(self.proc,), daemon=True).start()

        self.start_btn.config(state="disabled")
        self.entry.config(state="disabled")
        self.stop_btn.config(state="normal")

    def _pump_proc_output(self, proc):
        """서브프로세스 stdout(+stderr 합침)을 한 줄씩 읽어 큐에 적재 —
        tkinter 위젯은 메인스레드에서만 건드려야 하므로 update_status()가
        폴링해서 옮겨감."""
        for line in iter(proc.stdout.readline, ""):
            self.log_queue.put(line)
        proc.stdout.close()

    def stop_mission(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
        self.proc = None
        self.start_btn.config(state="normal")
        self.entry.config(state="normal")
        self.stop_btn.config(state="disabled")

    def update_status(self):
        while True:
            try:
                self._append_log(self.log_queue.get_nowait())
            except queue.Empty:
                break

        # 서브프로세스가 스스로 끝났으면(완주/크래시 등) 버튼 상태를 되돌림
        if self.proc is not None and self.proc.poll() is not None:
            self._append_log(f"=== 프로세스 종료 (exit code {self.proc.poll()}) ===\n")
            self.proc = None
            self.start_btn.config(state="normal")
            self.entry.config(state="normal")
            self.stop_btn.config(state="disabled")

        t = self.telemetry
        self.state_label.config(text=t.state_text)
        pos_text = f"({t.world_x:.2f}, {t.world_y:.2f})" if t.local_position_received else "—"
        self.pos_label.config(text=pos_text)
        self.target_label.config(
            text=f"({t.target_xy[0]:.1f}, {t.target_xy[1]:.1f})" if t.target_xy else "—")
        self.arm_label.config(
            text="ARMED" if t.armed else "DISARMED",
            foreground="#1a8a1a" if t.armed else "#b22222")
        self.offboard_label.config(
            text="ON" if t.offboard else "OFF",
            foreground="#1a8a1a" if t.offboard else "#b22222")

        self.root.after(STATUS_POLL_MS, self.update_status)


def main():
    rclpy.init()
    telemetry = TelemetryNode()
    threading.Thread(target=rclpy.spin, args=(telemetry,), daemon=True).start()

    root = tk.Tk()
    app = MissionGUI(root, telemetry)

    def on_close():
        if app.proc is not None and app.proc.poll() is None:
            if not messagebox.askyesno("종료", "미션이 실행 중입니다. 그래도 종료할까요?"):
                return
            app.proc.terminate()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
