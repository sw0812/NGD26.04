import cv2

cap = cv2.VideoCapture(0)  # HDMI 캡처 장치 인덱스 확인 필수

# 해상도 설정
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# FPS 설정
cap.set(cv2.CAP_PROP_FPS, 30)

# 윈도우 설정
cv2.namedWindow('WIRIS via HDMI', cv2.WINDOW_NORMAL)
cv2.resizeWindow('WIRIS via HDMI', 1280, 720)


while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break
    cv2.imshow('WIRIS via HDMI', frame)
    if cv2.waitKey(1) == 27:  # ESC 키 누르면 종료
        break

cap.release()
cv2.destroyAllWindows()