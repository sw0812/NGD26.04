#!/usr/bin/env python3
"""
카메라 2대로 라인트레이싱 신뢰도 교차검증 - 독립 실행 프로토타입.

역할 분리:
  - MAIN  (/dev/video0): search_phase_node.py가 실제로 쓰는 것과 동일한
    LineTracker 결과. 이 값만 비행 제어에 쓰인다고 가정.
  - SERVE (/dev/video2): 검증용. 결과를 제어에 절대 쓰지 않고, MAIN과
    비교해서 "지금 MAIN이 보는 라인을 믿을만한가"만 판단.

비교 기준 (baseline 때문에 e_y는 두 카메라가 원래부터 다르게 나오므로 비교 X):
  - line_found 일치 여부 (한쪽만 라인을 봤다 = 의심)
  - e_psi(각도) 차이가 ANGLE_AGREE_THRESH 이내인가 (같은 선이면 각도는 거의 같아야 함)

불일치가 MISMATCH_LOG_HOLD 프레임 넘게 지속되면 경고 로그만 남김 - 자동으로
미션을 멈추지는 않음(이 프로젝트의 기존 방침: 성가신 자동개입보다 로그+사람의
강제착륙 버튼을 최종 안전장치로 둠).

PX4/ROS2 연결 없음 - 순수 비전 로직만 독립적으로 돌리는 프로토타입.

실행: python3 test_dual_line_confidence.py
종료: q 또는 ESC
"""
import math
import sys
import time

import cv2

sys.path.insert(0, "/home/sasllab/ros2_ws/src/my_first_pkg")
from line_tracker import LineTracker  # noqa: E402

MAIN_DEV = "/dev/video0"
SERVE_DEV = "/dev/video2"
WIDTH, HEIGHT = 640, 480

ANGLE_AGREE_THRESH = math.radians(15)
MISMATCH_LOG_HOLD = 10  # 이 프레임 수 넘게 연속 불일치면 경고 로그


def open_cam(dev):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise SystemExit(f"카메라 열기 실패: {dev}")
    return cap


def draw_single(frame, tracker, result, label):
    h, w = frame.shape[:2]
    y0, y1 = int(h * tracker.roi_top_ratio), int(h * tracker.roi_bottom_ratio)
    vis = frame.copy()
    cv2.rectangle(vis, (0, y0), (w - 1, y1), (255, 200, 0), 2)
    if result.line_found:
        color = (0, 0, 255) if result.intersection else (0, 255, 0)
        cx = int(w / 2.0 + result.e_y)
        cv2.circle(vis, (cx, y1 - 5), 6, color, -1)
        text = f"e_y={result.e_y:+.1f}px e_psi={math.degrees(result.e_psi):+.1f}deg"
    else:
        color = (0, 165, 255)
        text = "LINE NOT FOUND"
    cv2.putText(vis, f"[{label}] {text}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, color, 2)
    return vis


def evaluate_agreement(r_main, r_serve):
    """반환: (agree: bool, reason: str)"""
    if r_main.line_found != r_serve.line_found:
        return False, "found-mismatch"
    if not r_main.line_found:
        return True, "both-not-found"
    diff = abs(r_main.e_psi - r_serve.e_psi)
    if diff > ANGLE_AGREE_THRESH:
        return False, f"angle-diff={math.degrees(diff):.1f}deg"
    return True, f"angle-diff={math.degrees(diff):.1f}deg"


def main():
    cap_main = open_cam(MAIN_DEV)
    cap_serve = open_cam(SERVE_DEV)
    tracker_main = LineTracker()
    tracker_serve = LineTracker()

    win = "Dual Line Confidence (q/ESC quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    mismatch_streak = 0

    # 평가용 통계 - "지금 잘 맞고 있는지" 수치로 보려고 추가
    # (2026-10-26: 경고 로그만으로는 평가가 안 돼서).
    stats_window_start = time.time()
    stats_total = 0
    stats_agree = 0
    stats_angle_diffs = []

    try:
        while True:
            ok_m, frame_m = cap_main.read()
            ok_s, frame_s = cap_serve.read()
            if not ok_m or not ok_s:
                print("프레임 읽기 실패 (카메라 연결 확인)")
                time.sleep(0.1)
                continue

            r_main = tracker_main.process(frame_m)
            r_serve = tracker_serve.process(frame_s)
            agree, reason = evaluate_agreement(r_main, r_serve)

            stats_total += 1
            if agree:
                stats_agree += 1
            if r_main.line_found and r_serve.line_found:
                stats_angle_diffs.append(abs(math.degrees(r_main.e_psi - r_serve.e_psi)))

            if agree:
                mismatch_streak = 0
            else:
                mismatch_streak += 1
                if mismatch_streak == MISMATCH_LOG_HOLD:
                    print(f"[WARN] MAIN/SERVE 불일치 {MISMATCH_LOG_HOLD}프레임 연속 "
                          f"({reason}) - MAIN 값은 계속 신뢰하되 점검 필요")

            if time.time() - stats_window_start >= 3.0:
                rate = 100.0 * stats_agree / max(1, stats_total)
                avg_diff = (sum(stats_angle_diffs) / len(stats_angle_diffs)
                            if stats_angle_diffs else float("nan"))
                print(f"[STATS] {stats_total}프레임 중 일치 {rate:.0f}% "
                      f"(양쪽 검출된 프레임 평균 각도차 {avg_diff:.1f}deg, "
                      f"n={len(stats_angle_diffs)})")
                stats_window_start = time.time()
                stats_total = stats_agree = 0
                stats_angle_diffs = []

            vis_m = draw_single(frame_m, tracker_main, r_main, "MAIN")
            vis_s = draw_single(frame_s, tracker_serve, r_serve, "SERVE")
            combined = cv2.hconcat([vis_m, vis_s])

            banner_color = (0, 200, 0) if agree else (0, 0, 255)
            banner_text = f"{'AGREE' if agree else 'DISAGREE'} ({reason})"
            cv2.rectangle(combined, (0, HEIGHT - 30), (combined.shape[1], HEIGHT),
                          (0, 0, 0), -1)
            cv2.putText(combined, banner_text, (10, HEIGHT - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, banner_color, 2)

            cv2.imshow(win, combined)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        cap_main.release()
        cap_serve.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
