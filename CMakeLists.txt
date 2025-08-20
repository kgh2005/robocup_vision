cmake_minimum_required(VERSION 3.10)
project(robocup_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(humanoid_interfaces REQUIRED)

find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(msg_files
  "msg/BoundingBox.msg"
  "msg/PanTilt.msg"
  "msg/PanTiltMsgs.msg"
  "msg/PanTiltStatusMsgs.msg"
)

# 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs builtin_interfaces sensor_msgs
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 인터페이스를 사용하는 타겟에 대한 설정 (executable보다 먼저 정의)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# 실행 파일
# ============================================================
add_executable(pan_tilt_camera_node
  src/usb_cam/insta/pan_tilt_camera_node.cpp
  src/usb_cam/insta/usb_camera.cpp)

ament_target_dependencies(pan_tilt_camera_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  rclcpp_lifecycle
)

target_link_libraries(pan_tilt_camera_node
  ${OpenCV_LIBRARIES}
  "${cpp_typesupport_target}"
)

install(TARGETS
  pan_tilt_camera_node
  DESTINATION lib/${PROJECT_NAME}
)
# ============================================================
add_executable(detection_node
  src/detection/detection_node.cpp
)

ament_target_dependencies(detection_node
  rclcpp
  sensor_msgs
  cv_bridge
  std_msgs
)

target_link_libraries(detection_node
  openvino::runtime
  ${OpenCV_LIBRARIES}
  "${cpp_typesupport_target}"
)

install(TARGETS
  detection_node
  DESTINATION lib/${PROJECT_NAME}
)
# ============================================================
add_executable(refiner_node
  src/refiner/refiner_node.cpp
)

ament_target_dependencies(refiner_node
  rclcpp
  sensor_msgs
  cv_bridge
  std_msgs
  humanoid_interfaces
)

target_link_libraries(refiner_node
  ${OpenCV_LIBRARIES}
  "${cpp_typesupport_target}"
)

install(TARGETS
  refiner_node
  DESTINATION lib/${PROJECT_NAME}
)
# ============================================================
add_executable(pan_tilt_node
  src/pan_tilt/pan_tilt_node.cpp
)

ament_target_dependencies(pan_tilt_node
  rclcpp
  std_msgs
  humanoid_interfaces
)

target_link_libraries(pan_tilt_node
  ${OpenCV_LIBRARIES}
  "${cpp_typesupport_target}"
)

install(TARGETS
  pan_tilt_node
  DESTINATION lib/${PROJECT_NAME}
)
# ============================================================

install(DIRECTORY 
  model
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
