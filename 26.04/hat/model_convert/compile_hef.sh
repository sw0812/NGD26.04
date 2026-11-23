#!/usr/bin/env bash
# x86 Linux PC + Hailo Dataflow Compiler(DFC, Hailo Developer Zone에서 다운로드, 별도 venv) 에서 실행.
# 사용: ./compile_hef.sh best.onnx calib_set.npy
# Hailo-10H 대상: --hw-arch hailo10h  (Hailo-8/8L용 hailo8/hailo8l 아님!)
set -euo pipefail
ONNX=${1:-best.onnx}; CALIB=${2:-calib_set.npy}; NAME=landing_pad

# 1) 파싱 (YOLOv8n 끝단 노드 이름은 netron으로 확인; 아래는 hailo_model_zoo yolov8n 기본값)
hailo parser onnx "$ONNX" --net-name $NAME --hw-arch hailo10h \
  --start-node-names images \
  --end-node-names /model.22/cv2.0/cv2.0.2/Conv /model.22/cv3.0/cv3.0.2/Conv \
                   /model.22/cv2.1/cv2.1.2/Conv /model.22/cv3.1/cv3.1.2/Conv \
                   /model.22/cv2.2/cv2.2.2/Conv /model.22/cv3.2/cv3.2.2/Conv

# 2) 양자화 + NMS 후처리 (nc=1 이므로 nms 설정의 classes를 1로)
cat > $NAME.alls <<'ALLS'
normalization1 = normalization([0.0, 0.0, 0.0], [255.0, 255.0, 255.0])
nms_postprocess("yolov8n_nms_config.json", meta_arch=yolov8, engine=cpu)
ALLS
cat > yolov8n_nms_config.json <<'JSON'
{"nms_scores_th": 0.2, "nms_iou_th": 0.7, "image_dims": [640, 640], "max_proposals_per_class": 20,
 "classes": 1, "regression_length": 16, "background_removal": false, "background_removal_index": 0,
 "bbox_decoders": [
  {"name": "bbox_decoder_41", "stride": 8,  "reg_layer": "conv41", "cls_layer": "conv42"},
  {"name": "bbox_decoder_52", "stride": 16, "reg_layer": "conv52", "cls_layer": "conv53"},
  {"name": "bbox_decoder_62", "stride": 32, "reg_layer": "conv62", "cls_layer": "conv63"}]}
JSON
# ※ reg_layer/cls_layer 이름(convNN)은 parser 후 생성된 ${NAME}.har 의 레이어 이름으로 맞출 것.

hailo optimize $NAME.har --hw-arch hailo10h --calib-set-path "$CALIB" --model-script $NAME.alls
hailo compiler ${NAME}_optimized.har --hw-arch hailo10h
echo "생성: ${NAME}.hef  -> Pi로 복사"
