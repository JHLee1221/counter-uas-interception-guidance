#ifndef COUNTER_UAS_TRACKING_GLOBAL_HEADER_H
#define COUNTER_UAS_TRACKING_GLOBAL_HEADER_H

#define DEG2RAD = M_PI / 180.0f;
#define RAD2DEG = 180.0f / M_PI;

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

// for using eigen library
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// for publishing and subscribing to images in ROS
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

// for using standard messages, float 64 type
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/header.hpp>

// for using vision messages, detection2d type
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

// for using mavros messages, float 32 type
#include <mavros_msgs/msg/mount_control.hpp>

// for using geometry messages
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

// for using navigation messages
#include <nav_msgs/msg/odometry.hpp> 

// for using visualization messages
#include <visualization_msgs/msg/marker.hpp>

// for using custom messages, w.r.t. bounding boxes
#include "counter_uas_imgproc/msg/bounding_box.hpp"
#include "counter_uas_imgproc/msg/bounding_boxes.hpp"
#include "counter_uas_imgproc/msg/illegal_drone_status.hpp"

using namespace std;
using namespace cv;

typedef struct
{
  Point ptCen;
  int nWidth;
  int nHeight;
  string strLabel;
} TrcInfo;

typedef struct
{
  float panCmd;
  float tiltCmd;
} CmdInfo;
#endif // COUNTER_UAS_TRACKING_GLOBAL_HEADER_H