# upscale_dataset.py
from PIL import Image
import glob, os

SCALE_H = 6  # 64 -> 384
SCALE_W = 2  # 360 -> 720 (종횡비 어느정도 유지, 필요시 조정)

src_dir = os.path.expanduser("~/lidar_dataset_yolo/images")
for split in ["train", "val"]:
    for f in glob.glob(f"{src_dir}/{split}/*.png"):
        im = Image.open(f)
        w, h = im.size
        im_big = im.resize((w*SCALE_W, h*SCALE_H), Image.NEAREST)  # depth map은 NEAREST 권장(블러 방지)
        im_big.save(f)

print("완료")

