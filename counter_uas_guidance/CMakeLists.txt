cmake_minimum_required(VERSION 3.8)
project(counter_uas_guidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rosidl_default_generators REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

# using eigen3
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

set (msg_files
  "msg/GimbalMode.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs sensor_msgs vision_msgs geometry_msgs 
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(typesupport_target_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  include
  include/yaw_rate_ctrl
  lib
  lib/yaw_rate_ctrl
  src
  ${BOOST_INCLUDE_DIRS}
)

install(DIRECTORY 
        config        
        include
        launch
        lib
        msg
        src
        DESTINATION share/${PROJECT_NAME}
)

install(FILES
  DESTINATION share/${PROJECT_NAME}
)

add_executable(counter_uas_offboard_ctrl_node src/main_counter_uas_offboard_ctrl_node.cpp
  include/global_header.h
  include/config_param_offboard_ctrl.h
  include/counter_uas_offboard_ctrl_lib.h
  include/yaw_rate_ctrl/yaw_rate_ctrl.h
  lib/config_param_offboard_ctrl.cpp
  lib/counter_uas_offboard_ctrl_lib.cpp
  lib/yaw_rate_ctrl/yaw_rate_ctrl.cpp
)
target_link_libraries(counter_uas_offboard_ctrl_node ${typesupport_target_cpp} ${EIGEN3_LIBRARIES})
ament_target_dependencies(counter_uas_offboard_ctrl_node rclcpp rclpy geometry_msgs sensor_msgs vision_msgs std_msgs mavros_msgs nav_msgs Eigen3 tf2 tf2_geometry_msgs)

install(TARGETS
  counter_uas_offboard_ctrl_node

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
