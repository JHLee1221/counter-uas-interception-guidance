#ifndef SIYI_EO_ROS2_GLOBAL_HEADER_H
#define SIYI_EO_ROS2_GLOBAL_HEADER_H

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
#include <stdexcept>
#include <utility>
#include <atomic>
#include <fcntl.h>
#include <termios.h>
#include <csignal>

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

// essential header for ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/qos.hpp>

// for using OpenCV library
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xphoto.hpp>
#include <opencv2/tracking.hpp>
#include <cv_bridge/cv_bridge.h>

// for publishing and subscribing to images in ROS
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

// for using standard messages, float 64 type
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/header.hpp>

#endif // SIYI_EO_ROS2_GLOBAL_HEADER_H