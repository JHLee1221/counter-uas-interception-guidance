#ifndef COUNTER_UAS_GUIDANCE_GLOBAL_HEADER_H
#define COUNTER_UAS_GUIDANCE_GLOBAL_HEADER_H

// using vector type data
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <signal.h>
#include <ctime>
#include <cstring>
#include <vector>
#include <dirent.h>
#include <fstream>
#include <random>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <algorithm>
#include <memory>
#include <cmath>
#include <sstream>
#include <numeric>

// for using folder generation
#include <sys/stat.h>
#include <sys/types.h>

// for using boost asio library
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <boost/date_time/local_time/local_time_io.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>

// essential header for ROS-OpenCV operation
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

// for using eigen library
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// for using standard messages, float 64 type
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/header.hpp>

// for using mavros messages
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/mount_control.hpp>

// for using navigation messages
#include <nav_msgs/msg/odometry.hpp>

#define PI 3.141592
#define R2D 180.0 / PI
#define D2R PI / 180.0