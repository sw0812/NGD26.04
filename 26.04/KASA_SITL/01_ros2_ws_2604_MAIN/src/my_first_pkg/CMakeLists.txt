cmake_minimum_required(VERSION 3.8)
project(my_first_pkg)

# C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------
# Dependencies
# -------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
# -------------------------------
# Service
# -------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
 "action/Countdown.action" 
 "srv/AddTwoInts.srv"
)

ament_export_dependencies(rosidl_default_runtime)

# -------------------------------
# Executables
# -------------------------------
add_executable(publisher src/publisher.cpp)
add_executable(subscriber src/subscriber.cpp)
add_executable(server src/server.cpp)
add_executable(client src/client.cpp)
add_executable(parameter src/parameter.cpp)
add_executable(server_action src/server_action.cpp)
add_executable(client_action src/client_action.cpp)
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
add_executable(static_tf_broadcaster src/static_tf_broadcaster.cpp)
add_executable(tf_listener src/tf_listener.cpp)
add_executable(aruco_detector src/aruco_detector.cpp
)
# -------------------------------
# Link libraries
# -------------------------------
ament_target_dependencies(publisher
  rclcpp
  std_msgs
)

ament_target_dependencies(subscriber
  rclcpp
  std_msgs
)

ament_target_dependencies(server
  rclcpp
  std_msgs
)
rosidl_target_interfaces(server
  ${PROJECT_NAME} 
  "rosidl_typesupport_cpp"
)
ament_target_dependencies(client
  rclcpp
)

rosidl_target_interfaces(client
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

ament_target_dependencies(parameter
  rclcpp
)

rosidl_target_interfaces(parameter
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

ament_target_dependencies(server_action
  rclcpp
  rclcpp_action
)

rosidl_target_interfaces(server_action
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
ament_target_dependencies(client_action
  rclcpp
  rclcpp_action
)

rosidl_target_interfaces(client_action
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
ament_target_dependencies(tf_broadcaster
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)
rosidl_target_interfaces(tf_broadcaster
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
ament_target_dependencies(static_tf_broadcaster
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)
rosidl_target_interfaces(static_tf_broadcaster
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
ament_target_dependencies(tf_listener
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)
rosidl_target_interfaces(tf_listener
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
ament_target_dependencies(
  aruco_detector

  rclcpp

  sensor_msgs

  cv_bridge
)

target_link_libraries(
  aruco_detector

  ${OpenCV_LIBS}
)
ament_target_dependencies(aruco_detector
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(aruco_detector
  ${OpenCV_LIBS}
)

# -------------------------------
# Install
# -------------------------------
install(TARGETS
  publisher
  subscriber
  server
  client
  parameter
  server_action
  client_action
  tf_broadcaster
  static_tf_broadcaster
  tf_listener
  aruco_detector
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------
# Test
# -------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# -------------------------------
# Package
# -------------------------------
# Install launch files
install(
  DIRECTORY
  launch
  urdf
  meshes
  aruco_detector
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
