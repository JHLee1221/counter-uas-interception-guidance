cmake_minimum_required(VERSION 3.8)
project(siyi_a8mini)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ 표준 지정
set(CMAKE_CXX_STANDARD 14)

# 공통 include 경로 설정
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 메인 노드
# add_executable(siyi_a8mini src/siyi_a8mini.cpp)
# target_include_directories(siyi_a8mini PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(siyi_a8mini PUBLIC c_std_99 cxx_std_17)

# 짐벌 제어 노드
add_executable(gimbal_control_node src/gimbal_control_node.cpp)
target_include_directories(gimbal_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(gimbal_control_node
  rclcpp
  sensor_msgs
)

# 영상 스트리밍 노드
add_executable(gimbal_stream_node src/gimbal_stream_node.cpp)
target_include_directories(gimbal_stream_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(gimbal_stream_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# 실행 파일 설치
install(TARGETS
  # siyi_a8mini
  gimbal_control_node
  gimbal_stream_node
  DESTINATION lib/${PROJECT_NAME})

# # 헤더 및 include 파일 설치
# install(DIRECTORY include/ DESTINATION include)

# launch 파일 생성
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# # yaml파일 설치
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/
# )

# 테스트 (옵션)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
