cmake_minimum_required(VERSION 3.8)
project(counter_uas_imgproc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set CUDA
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -O3 --use_fast_math")
set(CMAKE_CUDA_ARCHITECTURES 86)

enable_language(CUDA)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

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

# find_package(OpenCV 4.5 REQUIRED COMPONENTS
#     opencv_core
#     opencv_imgproc
#     opencv_highgui
#     opencv_videoio
#     opencv_dnn
#   CONFIG
# )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

# using CUDA
find_package(CUDA REQUIRED) 

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

set(msg_files
  "msg/BoundingBox.msg"
  "msg/BoundingBoxes.msg"
  "msg/IllegalDroneStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs sensor_msgs vision_msgs geometry_msgs 
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(typesupport_target_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  include
  include/misc
  include/yolov8_p2_lib
  include/yolov8_p2_infer_engine
  #include/particle_filter_tracking
  #include/lstm_tracking
  lib
  lib/misc
  lib/yolov8_p2_lib
  lib/yolov8_p2_infer_engine
  #lib/particle_filter_tracking
  #lib/lstm_tracking
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
        msg
        #scripts
        src
        #srv
        DESTINATION share/${PROJECT_NAME}
)

install(FILES
  DESTINATION share/${PROJECT_NAME}
)

# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("embed_platform on")
  include_directories(/usr/local/cuda/targets/aarch64-linux/include)
  link_directories(/usr/local/cuda/targets/aarch64-linux/lib)
else()
  message("embed_platform off")

  # cuda
  include_directories(/usr/local/cuda/include)
  link_directories(/usr/local/cuda/lib64)

  # tensorrt
  include_directories(/home/jhlee/TensorRT-8.5.2.2/include)
  link_directories(/home/jhlee/TensorRT-8.5.2.2/lib)
endif()

message(STATUS "${OpenCV_INCLUDE_DIRS}")
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${BOOST_LIBRARY_DIRS})
add_definitions(${BOOST_DEFINITIONS})

cuda_add_library(yolo_layer_plugin SHARED ${PROJECT_SOURCE_DIR}/include/yolov8_p2_lib/yololayer.cu)
target_link_libraries(yolo_layer_plugin nvinfer cudart)

file(GLOB_RECURSE YOLOV8_P2_SRCUS ${PROJECT_SOURCE_DIR}/lib/yolov8_p2_lib/*.cpp ${PROJECT_SOURCE_DIR}/lib/yolov8_p2_lib/*.cu)

# inference node
add_executable(counter_uas_yolov8p2_infer_node src/main_counter_uas_yolov8p2_infer_node.cpp
  include/counter_uas_yolov8p2_infer_lib.h
  lib/counter_uas_yolov8p2_infer_lib.cpp
  include/yolov8_p2_infer_engine/yolov8_p2_infer_engine.h
  lib/yolov8_p2_infer_engine/yolov8_p2_infer_engine.cpp
	include/misc/misc_calc_lib.h
	lib/misc/misc_calc_lib.cpp
	include/misc/misc_func.h
	lib/misc/misc_func.cpp
	include/misc/colormat.h
	include/config_param_detection.h 
	lib/config_param_detection.cpp 
	include/global_header.h 
  ${YOLOV8_P2_SRCUS}
)
target_link_libraries(counter_uas_yolov8p2_infer_node ${typesupport_target_cpp} ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${EIGEN_LIBRARIES} ${CUDA_LIBRARIES} nvinfer cudart yolo_layer_plugin)
ament_target_dependencies(counter_uas_yolov8p2_infer_node rclcpp sensor_msgs std_msgs geometry_msgs vision_msgs cv_bridge message_filters)

# #tracking node
# add_executable(kari_pf_track_node src/main_kari_rd_pf_track_node.cpp
#   include/kari_rd_pf_track_lib.h
#   lib/kari_rd_pf_track_lib.cpp
#   include/particle_filter_tracking/tracking_lib.h
#   lib/particle_filter_tracking/tracking_lib.cpp
#   include/particle_filter_tracking/particle_filter_lib.h
#   lib/particle_filter_tracking/particle_filter_lib.cpp  
# 	include/misc/misc_calc_lib.h
# 	lib/misc/misc_calc_lib.cpp
# 	include/misc/misc_func.h
# 	lib/misc/misc_func.cpp
# 	include/misc/colormat.h
# 	include/config_param.h 
# 	lib/config_param.cpp 
# 	include/global_header.h 
# )
# target_link_libraries(kari_pf_track_node ${typesupport_target_cpp} ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${EIGEN_LIBRARIES})
# ament_target_dependencies(kari_pf_track_node rclcpp sensor_msgs std_msgs geometry_msgs vision_msgs cv_bridge message_filters)

# add_executable(kari_multi_pf_track_node src/main_kari_multi_pf_track_node.cpp
#   include/kari_rd_multi_pf_track_lib.h
#   lib/kari_rd_multi_pf_track_lib.cpp
#   include/particle_filter_tracking/multi_tracking_lib.h
#   lib/particle_filter_tracking/multi_tracking_lib.cpp
#   include/particle_filter_tracking/particle_filter_lib.h
#   lib/particle_filter_tracking/particle_filter_lib.cpp  
#   include/particle_filter_tracking/hungarian_algorithm_lib.h
#   lib/particle_filter_tracking/hungarian_algorithm_lib.cpp
# 	include/misc/misc_calc_lib.h
# 	lib/misc/misc_calc_lib.cpp
# 	include/misc/misc_func.h
# 	lib/misc/misc_func.cpp
# 	include/misc/colormat.h
# 	include/config_param.h 
# 	lib/config_param.cpp 
# 	include/global_header.h 
# )
# target_link_libraries(kari_multi_pf_track_node ${typesupport_target_cpp} ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${EIGEN_LIBRARIES})
# ament_target_dependencies(kari_multi_pf_track_node rclcpp sensor_msgs std_msgs geometry_msgs vision_msgs cv_bridge message_filters)

# add_executable(kari_lstm_track_node src/main_kari_lstm_track_node.cpp
#   include/kari_rd_lstm_track_lib.h
#   lib/kari_rd_lstm_track_lib.cpp
#   include/lstm_tracking/kari_rd_lstm_engine_lib.h
#   lib/lstm_tracking/kari_rd_lstm_engine_lib.cpp
#   include/lstm_tracking/kari_rd_lstm_minmaxscaler_lib.h
#   lib/lstm_tracking/kari_rd_lstm_minmaxscaler_lib.cpp  
# 	include/misc/misc_calc_lib.h
# 	lib/misc/misc_calc_lib.cpp
# 	include/misc/misc_func.h
# 	lib/misc/misc_func.cpp
# 	include/misc/colormat.h
# 	include/config_param.h 
# 	lib/config_param.cpp 
# 	include/global_header.h 
# )
# target_link_libraries(kari_lstm_track_node ${typesupport_target_cpp} ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${EIGEN_LIBRARIES} ${CUDA_LIBRARIES} nvinfer cudart nvinfer_plugin curand cudnn)
# ament_target_dependencies(kari_lstm_track_node rclcpp sensor_msgs std_msgs geometry_msgs vision_msgs cv_bridge message_filters)

install(TARGETS
  #kari_rd_infer_raw_test_node
  #ros2_test_node
  counter_uas_yolov8p2_infer_node
  # kari_pf_track_node
  # kari_multi_pf_track_node
  # kari_lstm_track_node

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
