#!/usr/bin/env python3
"""양자화(INT8) 캘리브레이션용 이미지 준비.
학습 데이터셋(lidar_dataset_yolo*.zip)의 360x64 depth 렌더 이미지를 추론 노드와 동일하게
letterbox(imgsz x imgsz, 회색 114 패딩)해서 npy 한 개로 저장한다.
  python3 make_calib.py <dataset_images_dir> calib_set.npy 640 [개수=1024]
* 이미지가 거의 비어 있는(≈108바이트짜리 PNG = 패드 없음) 프레임만 모으면 양자화가 망가지므로
  패드가 보이는 프레임(파일 크기가 큰 것) 위주로 섞어 뽑는다."""
import sys, glob, os, random
import numpy as np, cv2

src, out = sys.argv[1], sys.argv[2]
imgsz = int(sys.argv[3]) if len(sys.argv) > 3 else 640
n = int(sys.argv[4]) if len(sys.argv) > 4 else 1024

def letterbox(img, size):
    h, w = img.shape[:2]
    s = size / max(h, w)
    nh, nw = int(round(h * s)), int(round(w * s))
    img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, np.uint8)
    top, left = (size - nh) // 2, (size - nw) // 2
    canvas[top:top + nh, left:left + nw] = img
    return canvas

files = sorted(glob.glob(os.path.join(src, '*.png')))
full = [f for f in files if os.path.getsize(f) > 1000]    # 패드/장애물 있는 프레임
empty = [f for f in files if os.path.getsize(f) <= 1000]
random.seed(0)
pick = random.sample(full, min(len(full), int(n * 0.8))) + random.sample(empty, min(len(empty), n - int(n * 0.8)))
arr = np.stack([letterbox(cv2.cvtColor(cv2.imread(f, 0), cv2.COLOR_GRAY2RGB), imgsz) for f in pick])
np.save(out, arr)    # (N, imgsz, imgsz, 3) uint8 RGB
print(out, arr.shape, f'(패드프레임 {len(full)}장 중 사용)')
