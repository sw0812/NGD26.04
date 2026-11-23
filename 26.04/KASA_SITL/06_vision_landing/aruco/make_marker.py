import cv2

# ArUco Dictionary 선택
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Marker ID 0, 크기 600x600
marker = cv2.aruco.generateImageMarker(dictionary, 0, 600)

# 저장
cv2.imwrite("aruco0.png", marker)

print("ArUco marker saved!")
