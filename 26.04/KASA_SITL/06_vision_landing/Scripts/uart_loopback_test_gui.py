#!/usr/bin/env python3
"""
UART 루프백/원인진단 테스트 - GUI 버전
Jetson /dev/ttyTHS0 (8번=TX, 10번=RX) 테스트용.
"""
import datetime
import os
import struct
import sys
import termios
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext

LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "uart_loopback_test.log")


def open_raw(dev, baud):
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    baud_const = getattr(termios, f"B{baud}", None)
    if baud_const is None:
        raise ValueError(f"지원 안 하는 baud: {baud}")
    attrs[4] = baud_const
    attrs[5] = baud_const
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[0] = 0
    attrs[1] = 0
    attrs[3] = 0
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


class App:
    def __init__(self, root):
        self.root = root
        root.title("UART 루프백 진단 테스트")
        root.geometry("640x520")

        top = ttk.Frame(root, padding=10)
        top.pack(fill=tk.X)

        ttk.Label(top, text="장치:").grid(row=0, column=0, sticky="w")
        self.dev_var = tk.StringVar(value="/dev/ttyTHS0")
        ttk.Entry(top, textvariable=self.dev_var, width=20).grid(row=0, column=1, padx=5)

        ttk.Label(top, text="Baud:").grid(row=0, column=2, sticky="w")
        self.baud_var = tk.StringVar(value="57600")
        ttk.Entry(top, textvariable=self.baud_var, width=10).grid(row=0, column=3, padx=5)

        self.run_btn = ttk.Button(top, text="테스트 시작", command=self.start_test)
        self.run_btn.grid(row=0, column=4, padx=10)

        self.status_var = tk.StringVar(value="대기 중")
        self.status_label = ttk.Label(root, textvariable=self.status_var, font=("", 13, "bold"))
        self.status_label.pack(pady=(0, 5))

        self.text = scrolledtext.ScrolledText(root, wrap=tk.WORD, font=("monospace", 10))
        self.text.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        self.text.tag_configure("ok", foreground="#0a7d1e")
        self.text.tag_configure("fail", foreground="#c62828")
        self.text.tag_configure("warn", foreground="#e08a00")

    def log(self, msg, tag=None, f=None):
        self.text.insert(tk.END, msg + "\n", tag)
        self.text.see(tk.END)
        if f:
            f.write(msg + "\n")

    def start_test(self):
        self.run_btn.config(state=tk.DISABLED)
        self.status_var.set("테스트 진행 중...")
        self.status_label.configure(foreground="black")
        self.text.delete("1.0", tk.END)
        threading.Thread(target=self.run_test, daemon=True).start()

    def run_test(self):
        dev = self.dev_var.get().strip()
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            self.log("[FAIL] baud 값이 숫자가 아닙니다.", "fail")
            self.finish(False, "설정 오류")
            return

        with open(LOG_PATH, "a") as f:
            self.log(f"[{datetime.datetime.now().isoformat(timespec='seconds')}] 테스트 시작 (장치={dev}, baud={baud})", None, f)

            try:
                fd = open_raw(dev, baud)
            except Exception as e:
                self.log(f"[FAIL] 포트 열기/설정 실패: {e}", "fail", f)
                self.finish(False, "포트 열기 실패")
                return

            marker = b"PX4JETSON" + struct.pack("<I", int(time.time()) & 0xFFFFFFFF)
            total_sent = 0
            total_recv = b""
            rounds = 6

            for i in range(rounds):
                payload = marker + f"-{i:02d}".encode()
                os.write(fd, payload)
                total_sent += len(payload)
                time.sleep(0.3)
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    chunk = b""
                total_recv += chunk
                self.log(f"  round {i}: 전송 {len(payload)}B, 누적 수신 {len(total_recv)}B", None, f)

            os.close(fd)

            self.log("-" * 60, None, f)
            self.log(f"총 전송: {total_sent} bytes / 총 수신: {len(total_recv)} bytes", None, f)
            self.log("-" * 60, None, f)

            if len(total_recv) == 0:
                self.log("[진단] 수신 0바이트 -> 루프가 열려있음(끊김)", "fail", f)
                self.log("  원인 후보:", "fail", f)
                self.log("   - 8번(TX)-10번(RX) 점퍼가 실제로 연결 안 됨", "fail", f)
                self.log("   - Jetson TX(8번)가 신호를 안 내보냄", "fail", f)
                self.log("   - 포트/baud가 실제 배선과 안 맞음", "fail", f)
                self.finish(False, "실패: 수신 없음 (TX/점퍼 확인 필요)")
            elif marker[:10] in total_recv:
                self.log("[진단] 보낸 마커가 그대로 수신됨", "ok", f)
                self.log("  -> Jetson TX(8번)/RX(10번) 하드웨어 양방향 모두 정상", "ok", f)
                self.log("  -> Pixhawk 연결 시에도 안 되면, 문제는 Pixhawk까지 가는", "ok", f)
                self.log("     배선/커넥터 또는 Pixhawk RX 핀 쪽입니다.", "ok", f)
                self.finish(True, "성공: Jetson UART 하드웨어 정상")
            else:
                self.log("[진단] 일부 수신됐지만 보낸 것과 불일치", "warn", f)
                self.log("  -> baud 불일치 / 노이즈 / 접촉 불량 가능성", "warn", f)
                self.log(f"  수신 앞부분(hex): {total_recv[:80].hex()}", "warn", f)
                self.finish(None, "경고: 데이터 불일치 (배선 접촉/baud 확인)")

            self.log(f"[{datetime.datetime.now().isoformat(timespec='seconds')}] 테스트 종료, 로그: {LOG_PATH}", None, f)

    def finish(self, ok, status_text):
        def _update():
            self.status_var.set(status_text)
            if ok is True:
                self.status_label.configure(foreground="#0a7d1e")
            elif ok is False:
                self.status_label.configure(foreground="#c62828")
            else:
                self.status_label.configure(foreground="#e08a00")
            self.run_btn.config(state=tk.NORMAL)
        self.root.after(0, _update)


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
