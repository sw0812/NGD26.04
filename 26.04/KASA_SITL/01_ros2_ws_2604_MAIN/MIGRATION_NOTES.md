# Foxy → Lyrical Luth (Ubuntu 26.04) 마이그레이션 노트

원본: `~/ros2_ws/src/`

## 포함된 패키지 (네 코드)
- `my_first_pkg` (ament_cmake, C++ rclcpp 튜토리얼 노드 + 독립 실행 Python 스크립트들: search_phase_node.py, line_tracker.py, vision_odometry_node.py 등)
- `precision_landing`, `depth_processor`, `lidar_depth_pkg`, `aruco_detector` (전부 ament_python)
- `lidar_depth_pkg_old` — `COLCON_IGNORE`로 이미 빌드 제외된 죽은 코드, package.xml도 비어있음(`404: Not Found` 텍스트만 들어있음) — 그대로 복사만 하고 손 안 댐

## 포함 안 한 것
- `px4_msgs`, `ros2_aruco` — 네가 짠 코드 아니라 외부 git repo. Lyrical에서는 이 레포를 새로 clone해서 그쪽 브랜치/버전으로 다시 빌드하는 게 맞음(복사해서 포팅할 대상이 아님). `px4_msgs`는 특히 쓰는 PX4 펌웨어 버전이랑 메시지 정의가 맞아야 함.

## 실제로 찾은 문제 / 고친 것
- `my_first_pkg/CMakeLists.txt`: `cmake_minimum_required(VERSION 3.5)` → `3.8`로 올림. CMake 4.x가 3.5 미만을 지원 중단해서(정확히 3.5는 아직 괜찮지만) 여유 있게 올려둠.

## 코드 스캔해서 확인한 것 (문제 없음 — 이미 깨끗함)
- `distutils` 사용 없음 (Python 3.12에서 완전히 제거됨 — 다 `setuptools` 씀)
- `collections.Mapping` 류 구버전 ABC import 없음 (Python 3.10+에서 제거됨)
- 구버전 `cv2.aruco.Dictionary_get`/`DetectorParameters_create` 없음 (이미 `getPredefinedDictionary`/`generateImageMarker` 신버전 API 사용 중)
- 구버전 `RMW_QOS_POLICY_*` 긴 이름 enum 없음
- 모든 `package.xml`이 이미 `package_format3` (Lyrical에서도 유효)
- Python 버전 하드코딩(3.8 pin) 없음

## 한계 (여기서는 확인 불가능)
- 이 머신엔 ROS2 Lyrical이 설치되어 있지 않아서 실제 빌드(`colcon build`)는 못 해봤음 — 문법 스캔으로 찾을 수 있는 것만 확인함. 진짜 설치된 Lyrical 환경에서 `colcon build`를 돌려봐야 rclcpp/rclpy API 변경으로 인한 컴파일 에러까지 100% 잡힘.
- C++ 쪽(`my_first_pkg`의 rclcpp 튜토리얼 노드들)은 API가 매우 안정적이라 가능성 낮지만, 마찬가지로 실제 빌드 전까지는 확정 못 함.
