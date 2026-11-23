#!/usr/bin/env python3
"""
ArUco 마커(5X5_50, ID 0)를 A4 PDF로 출력용 생성.
aruco_tracker.yaml의 marker_dict(5X5_50) 설정과 일치시킴.

실행: python3 make_aruco_pdf.py
출력: ~/Desktop/aruco_marker_id0.pdf
"""
import cv2
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

pdfmetrics.registerFont(TTFont("NanumGothic", "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"))
pdfmetrics.registerFont(TTFont("NanumGothicBold", "/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf"))

OUT_PNG = "/tmp/aruco_marker_id0.png"
OUT_PDF = "/home/sasllab/Desktop/aruco_marker_id0.pdf"

MARKER_ID = 0
DICT_NAME = cv2.aruco.DICT_5X5_50
MARKER_SIZE_CM = 15.0  # 실측/조정하기 쉬운 크기. 필요시 여기 값만 바꾸면 됨.
PIXELS = 1000  # 고해상도로 생성 후 PDF에 삽입 (인쇄 시 계단현상 방지)

aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_NAME)
marker_img = cv2.aruco.generateImageMarker(aruco_dict, MARKER_ID, PIXELS)
cv2.imwrite(OUT_PNG, marker_img)

c = canvas.Canvas(OUT_PDF, pagesize=A4)
page_w, page_h = A4

# 마커를 페이지 중앙에 배치
x = (page_w - MARKER_SIZE_CM * cm) / 2
y = (page_h - MARKER_SIZE_CM * cm) / 2 + 1.5 * cm
c.drawImage(OUT_PNG, x, y, width=MARKER_SIZE_CM * cm, height=MARKER_SIZE_CM * cm)

# 안내 텍스트 (실제 크기 확인용 + 정보)
c.setFont("NanumGothicBold", 14)
c.drawCentredString(page_w / 2, y - 1.0 * cm, f"ArUco 5X5_50  -  ID {MARKER_ID}")
c.setFont("NanumGothic", 10)
c.drawCentredString(page_w / 2, y - 1.6 * cm,
                     f"한 변 {MARKER_SIZE_CM:.0f}cm  (인쇄 후 자로 재서 실제 크기 확인, "
                     f"프린터 '실제 크기/100%' 설정으로 인쇄할 것 - '페이지에 맞춤' 금지)")

# 눈금자(1cm) - 인쇄 배율이 맞는지 검증용
ruler_y = 1.5 * cm
c.setFont("NanumGothic", 8)
c.drawString(1.5 * cm, ruler_y + 0.3 * cm, "검증용 눈금자 (10cm)")
for i in range(11):
    xpos = 1.5 * cm + i * cm
    tick = 0.3 * cm if i % 5 == 0 else 0.15 * cm
    c.line(xpos, ruler_y, xpos, ruler_y + tick)
c.line(1.5 * cm, ruler_y, 1.5 * cm + 10 * cm, ruler_y)

c.save()
print(f"저장됨: {OUT_PDF}")
