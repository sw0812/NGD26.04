#!/usr/bin/env python3
"""C270 등 일반 웹캠으로 line_tracker.LineTracker 검출 품질을 눈으로 확인.

ROS2/PX4/시뮬레이션 없이 곧바로 실행 가능 (cv2 + numpy만 있으면 됨).
search_phase_node.py가 실제로 쓰는 것과 동일한 LineTracker를 그대로
가져다 쓰므로, 여기서 확인되는 결과가 곧 비행 중 노드가 보는 결과다.

사용법:
    python3 test_line_tracker_camera.py            # /dev/video0
    python3 test_line_tracker_camera.py --device 1  # /dev/video1
    q 또는 ESC로 종료. 's'로 현재 프레임+마스크 스냅샷 저장.
"""
import argparse
import time

import cv2

from line_tracker import LineTracker


def draw_overlay(frame, tracker, result, mask):
    h, w = frame.shape[:2]
    y0 = int(h * tracker.roi_top_ratio)
    y1 = int(h * tracker.roi_bottom_ratio)

    vis = frame.copy()
    cv2.rectangle(vis, (0, y0), (w - 1, y1), (255, 200, 0), 2)
    cv2.line(vis, (w // 2, y0), (w // 2, y1), (0, 0, 255), 1)

    if result.line_found:
        color = (0, 0, 255) if result.intersection else (0, 255, 0)
        cx = int(w / 2.0 + result.e_y)
        cy = y1 - 5
        cv2.circle(vis, (cx, cy), 6, color, -1)
        status = "INTERSECTION" if result.intersection else "LINE"
        text = f"{status}  e_y={result.e_y:+.1f}px  e_psi={result.e_psi:+.3f}rad"
    else:
        color = (0, 165, 255)
        text = "LINE NOT FOUND"

    cv2.putText(vis, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # 이진화 마스크를 우측 하단에 작게 얹어서 왜 검출/미검출인지 바로 보이게.
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    mh, mw = mask_bgr.shape[:2]
    scale = min(1.0, (w * 0.35) / mw)
    small = cv2.resize(mask_bgr, (int(mw * scale), int(mh * scale)))
    sh, sw = small.shape[:2]
    vis[y0:y0 + sh, w - sw:w] = small
    cv2.rectangle(vis, (w - sw, y0), (w - 1, y0 + sh), (255, 200, 0), 1)

    return vis


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", type=int, default=0, help="/dev/video 번호 (기본 0)")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise SystemExit(f"/dev/video{args.device} 열기 실패 — 연결/권한(video 그룹) 확인 필요")

    tracker = LineTracker()
    win = "line_tracker (q/ESC to quit, s to save)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("프레임 읽기 실패")
                time.sleep(0.1)
                continue

            h, w = frame.shape[:2]
            y0, y1 = int(h * tracker.roi_top_ratio), int(h * tracker.roi_bottom_ratio)
            gray = cv2.cvtColor(frame[y0:y1, :], cv2.COLOR_BGR2GRAY)
            mask = tracker._binarize(gray)
            result = tracker.process(frame)

            vis = draw_overlay(frame, tracker, result, mask)
            cv2.imshow(win, vis)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('s'):
                ts = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(f"line_tracker_{ts}.png", vis)
                print(f"saved line_tracker_{ts}.png")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
