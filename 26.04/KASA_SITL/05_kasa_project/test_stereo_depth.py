#!/usr/bin/env python3
"""
2대 웹캠(C270 x2) 스테레오 깊이 측정 프로토타입 UI.

아직 좌/우 스테레오 캘리브레이션(stereoCalibrate)은 안 한 상태라, 여기서는
camera_calibration.yaml의 모노 fx를 그대로 쓰고 좌/우 렌즈가 수평으로 나란히
(roll/pitch 정렬) 달려있다고 가정한다 -> 디스패리티 절대값은 참고용이고,
"베이스라인을 몇 cm로 하면 2m에서 오차가 얼마나 나오는지" 감 잡는 용도.

실행: python3 test_stereo_depth.py [--left /dev/video0] [--right /dev/video2]
조작:
  - 좌측(왼쪽 카메라) 화면 클릭 -> 그 지점의 깊이(m) 표시
  - 트랙바로 baseline(mm), numDisparities, blockSize 조정
  - 's' : 현재 좌/우 프레임을 snapshot_L.png / snapshot_R.png로 저장(추후 스테레오 캘리브레이션용)
  - 'q' : 종료
"""
import argparse
import time

import cv2
import numpy as np

# camera_calibration.yaml에서 가져온 모노 캘리브레이션 값 (1280x720 기준)
FX = 1371.19
FY = 1371.36
CX = 663.71
CY = 375.07


def open_cam(dev, width=1280, height=720):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    # YUYV(기본, 비압축)는 1280x720에서 7.5fps로 제한됨(USB 대역폭) -> MJPG로
    # 요청하면 같은 해상도에서 30fps까지 가능 (2026-10-26 실측, v4l2-ctl 확인).
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 오래된 프레임이 쌓여 밀리는 것 방지
    if not cap.isOpened():
        raise RuntimeError(f"카메라 열기 실패: {dev}")
    return cap


def make_matcher(num_disp, block_size):
    num_disp = max(16, (num_disp // 16) * 16)  # StereoSGBM 요구사항: 16의 배수
    block_size = block_size if block_size % 2 == 1 else block_size + 1
    block_size = max(5, block_size)
    return cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disp,
        blockSize=block_size,
        P1=8 * 3 * block_size ** 2,
        P2=32 * 3 * block_size ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
    )


class ClickState:
    def __init__(self):
        self.xy = None


def on_mouse(event, x, y, flags, state):
    if event == cv2.EVENT_LBUTTONDOWN:
        state.xy = (x, y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--left", default="/dev/video0")
    ap.add_argument("--right", default="/dev/video2")
    args = ap.parse_args()

    cap_l = open_cam(args.left)
    cap_r = open_cam(args.right)

    # QT 백엔드에서 창 이름에 한글(비 ASCII)이 들어가면 createTrackbar가
    # 그 창을 못 찾고 "NULL window handler" 에러를 냄 -> 2026-10-26 실측.
    # 창 제목은 ASCII로, 안내문구는 putText로 화면 안에 그려서 해결.
    win = "Stereo Depth Prototype"
    cv2.startWindowThread()
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.imshow(win, np.zeros((480, 640, 3), dtype=np.uint8))
    cv2.waitKey(30)
    cv2.createTrackbar("baseline(mm)", win, 60, 300, lambda v: None)
    cv2.createTrackbar("numDisparities/16", win, 6, 16, lambda v: None)
    cv2.createTrackbar("blockSize", win, 9, 31, lambda v: None)
    # 마우스 없이도 바로 꽉 찬 화면으로 보이게 (트랙바 만든 다음에 걸어야
    # 트랙바 생성이 안 막힘). 'f'로 토글 가능.
    fullscreen = True
    cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    click = ClickState()
    cv2.setMouseCallback(win, on_mouse, click)

    matcher = None
    matcher_key = None

    # 평가용 통계 - 마우스 없이도 매칭 품질을 수치로 보려고 추가
    # (2026-10-26: 클릭 평가가 안 되는 상황이라 추가).
    stats_window_start = time.time()
    stats_valid_ratios = []
    stats_center_depths = []

    while True:
        ok_l, frame_l = cap_l.read()
        ok_r, frame_r = cap_r.read()
        if not ok_l or not ok_r:
            print("프레임 읽기 실패 (카메라 연결 확인)")
            break

        h, w = frame_l.shape[:2]
        small_w, small_h = w // 2, h // 2

        # SGBM을 풀해상도(1280x720)로 돌리면 2.6fps밖에 안 나와서(실측,
        # 2026-10-26) 절반 해상도로 낮춰서 매칭 -> 18fps대로 캡처 속도를
        # 거의 따라감. 깊이 계산은 fx도 같이 절반으로 스케일하면 그대로 맞음
        # (depth = fx*B/d 에서 fx, d가 같은 비율로 줄어들어 결과는 동일).
        small_l = cv2.resize(frame_l, (small_w, small_h))
        small_r = cv2.resize(frame_r, (small_w, small_h))
        gray_l = cv2.cvtColor(small_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(small_r, cv2.COLOR_BGR2GRAY)
        fx_small = FX * 0.5

        baseline_mm = max(1, cv2.getTrackbarPos("baseline(mm)", win))
        num_disp = cv2.getTrackbarPos("numDisparities/16", win) * 16
        block_size = cv2.getTrackbarPos("blockSize", win)

        # StereoSGBM 생성 자체가 꽤 비싸서, 트랙바가 안 바뀌었으면 매 프레임
        # 새로 만들지 않고 재사용 (2026-10-26: 프레임 느리다는 피드백으로 추가).
        key = (num_disp, block_size)
        if matcher is None or key != matcher_key:
            matcher = make_matcher(num_disp, block_size)
            matcher_key = key
        disp = matcher.compute(gray_l, gray_r).astype(np.float32) / 16.0  # subpixel 보정, shape=(small_h, small_w)

        disp_vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX)
        disp_vis = cv2.applyColorMap(disp_vis.astype(np.uint8), cv2.COLORMAP_JET)

        valid = disp > 0.5
        stats_valid_ratios.append(100.0 * np.count_nonzero(valid) / valid.size)
        cy0, cx0 = small_h // 2, small_w // 2
        center_patch = disp[cy0 - 10:cy0 + 10, cx0 - 10:cx0 + 10]
        center_valid = center_patch[center_patch > 0.5]
        if center_valid.size > 0:
            stats_center_depths.append(fx_small * (baseline_mm / 1000.0) / float(center_valid.mean()))

        if time.time() - stats_window_start >= 3.0:
            avg_valid = sum(stats_valid_ratios) / len(stats_valid_ratios)
            if stats_center_depths:
                avg_depth = sum(stats_center_depths) / len(stats_center_depths)
                depth_str = f"{avg_depth:.3f}m (n={len(stats_center_depths)})"
            else:
                depth_str = "측정 불가(중앙 매칭 실패)"
            print(f"[STATS] 전체 매칭 유효비율 평균 {avg_valid:.0f}%, "
                  f"화면 중앙 깊이 {depth_str}")
            stats_window_start = time.time()
            stats_valid_ratios = []
            stats_center_depths = []

        # readout(왼쪽 화면 표시용)도 처음부터 small 크기로 만들어서, 클릭
        # 좌표(window 표시 공간 = small 크기)와 disp 인덱스가 그대로 맞게 함
        # -> 예전엔 풀해상도 이미지에 축소공간 좌표로 그려서 마커 위치가
        # 어긋나는 버그가 있었음(2026-10-26 발견, 같이 수정).
        readout = small_l.copy()
        if click.xy is not None:
            cx, cy = click.xy
            if 0 <= cy < small_h and 0 <= cx < small_w:
                d = disp[cy, cx]
                cv2.drawMarker(readout, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
                if d > 0.5:
                    depth_m = (fx_small * (baseline_mm / 1000.0)) / d
                    text = f"d={d:.1f}px depth={depth_m:.3f}m"
                else:
                    text = f"d={d:.1f}px (too small)"
                cv2.putText(readout, text, (10, small_h - 15), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 255), 1)

        cv2.putText(readout, f"baseline={baseline_mm}mm fx={FX:.0f}px", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        # OpenCV 기본 폰트(Hershey)는 한글 글리프가 없어서 네모로 깨짐 -> ASCII만 사용.
        cv2.putText(readout, "click=depth  s=save  f=fullscreen  q=quit", (10, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        top = np.hstack([readout, small_r])
        bottom = cv2.resize(disp_vis, (w, small_h))
        combined = np.vstack([top, bottom])

        # 디스패리티 컬러맵 범례 - 우측하단에 반투명 박스로 표시
        # (2026-10-26: 색 의미를 물어봐서 화면에 바로 적어두도록 추가).
        legend_lines = ["DISPARITY COLOR:", "red/orange = near",
                        "blue/dark  = far / no match"]
        lx0, ly0 = w - 300, h - 85
        overlay = combined.copy()
        cv2.rectangle(overlay, (lx0 - 10, ly0 - 20), (w - 10, h - 10), (0, 0, 0), -1)
        combined = cv2.addWeighted(overlay, 0.6, combined, 0.4, 0)
        for i, line in enumerate(legend_lines):
            cv2.putText(combined, line, (lx0, ly0 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow(win, combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            cv2.imwrite("snapshot_L.png", frame_l)
            cv2.imwrite("snapshot_R.png", frame_r)
            print("저장됨: snapshot_L.png, snapshot_R.png")
        elif key == ord("f"):
            fullscreen = not fullscreen
            cv2.setWindowProperty(
                win, cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)

    cap_l.release()
    cap_r.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
