cmake_minimum_required(VERSION 3.8)
project(siyi_eo_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# using opencv
find_package(OpenCV 4.7.0 REQUIRED COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_videoio
    opencv_xfeatures2d
    opencv_xphoto
    opencv_tracking
    CONFIG
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  include
  lib
  src
  ${BOOST_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include/opencv4    
)

install(DIRECTORY
        config
        include
        launch
        lib
        src
        DESTINATION share/${PROJECT_NAME}
)

install(FILES
  DESTINATION share/${PROJECT_NAME}
)

# camera status node
add_executable(siyi_eo_status_node src/main_siyi_eo_status_node.cpp
  include/global_header.h
  include/config_param_status.h
  include/siyi_eo_status_lib.h
  lib/config_param_status.cpp
  lib/siyi_eo_status_lib.cpp
)
target_link_libraries(siyi_eo_status_node ${typesupport_target_cpp} ${OpenCV_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(siyi_eo_status_node rclcpp sensor_msgs std_msgs cv_bridge)

install(TARGETS
  siyi_eo_status_node
  
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
