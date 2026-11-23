#!/usr/bin/env python3
"""best.pt -> ONNX (Hailo Dataflow Compiler 입력용). x86 PC(ultralytics 설치된 곳)에서 실행.
  python3 export_onnx.py ../08_datasets_models/best.pt 640
DFC는 고정 입력 크기가 필요하므로 imgsz를 정해서 export (dynamic=False, opset 11~13 권장)."""
import sys
from ultralytics import YOLO

weights = sys.argv[1] if len(sys.argv) > 1 else 'best.pt'
imgsz = int(sys.argv[2]) if len(sys.argv) > 2 else 640
path = YOLO(weights).export(format='onnx', imgsz=imgsz, opset=11, simplify=True, dynamic=False)
print('ONNX:', path)
