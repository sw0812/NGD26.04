#!/usr/bin/env python3
"""
구획선(착륙패드 경계) 검출 단독 테스트 스크립트.

ROS2/Agent/aruco_tracker 아무것도 안 띄우고, 웹캠만 바로 열어서
Canny+contour 기반 사각형 경계 검출을 실시간으로 확인합니다.

실행: python3 test_boundary_detection.py [--dev /dev/video0]
종료: 화면에서 'q' 키
"""
import argparse
import time

import cv2
import numpy as np


def detect_boundary(arr_bgr):
    """arr_bgr: OpenCV 기본 BGR 프레임. 사각형 경계 후보를 찾아 그려서 반환."""
    gray = cv2.cvtColor(arr_bgr, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    edges = cv2.dilate(edges, None, iterations=1)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    frame_area = gray.shape[0] * gray.shape[1]
    best, best_area = None, 0
    for cnt in contours:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            area = cv2.contourArea(approx)
            if area > 0.015 * frame_area and area > best_area:
                best, best_area = approx, area

    found = best is not None
    if found:
        cv2.polylines(arr_bgr, [best], True, (83, 168, 52), 3)  # BGR 초록
        m = cv2.moments(best)
        if m["m00"] != 0:
            cx, cy = int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])
            cv2.drawMarker(arr_bgr, (cx, cy), (83, 168, 52), cv2.MARKER_CROSS, 16, 2)
    return arr_bgr, found, edges


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/video0")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--show-edges", action="store_true", help="Canny 엣지 맵도 같이 보기")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.dev)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print(f"[FAIL] {args.dev} 열기 실패")
        return

    print("실행 중... 'q' 키로 종료")
    fps_t0 = time.time()
    frames = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[WARN] 프레임 읽기 실패")
            continue

        annotated, found, edges = detect_boundary(frame.copy())

        status = "BOUNDARY DETECTED" if found else "searching..."
        color = (83, 168, 52) if found else (53, 42, 217)  # BGR 초록/빨강
        cv2.rectangle(annotated, (0, 0), (annotated.shape[1] - 1, annotated.shape[0] - 1), color, 4)
        cv2.putText(annotated, status, (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

        frames += 1
        if time.time() - fps_t0 >= 1.0:
            fps = frames / (time.time() - fps_t0)
            print(f"fps={fps:.1f}  detected={found}")
            frames = 0
            fps_t0 = time.time()

        cv2.imshow("boundary detection test", annotated)
        if args.show_edges:
            cv2.imshow("edges", edges)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
