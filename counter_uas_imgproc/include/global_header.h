#ifndef COUNTER_UAS_IMGPROC_GLOBAL_HEADER_H
#define COUNTER_UAS_IMGPROC_GLOBAL_HEADER_H

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

// for using json library
#include <nlohmann/json.hpp>

// essential header for yolov8_p2 inference operation
#include "cuda_utils.h"
#include "logging.h"
#include "model.h"
#include "postprocess.h"
#include "preprocess.h"
#include "utils.h"
#include <NvInfer.h>
#include <cuda_runtime_api.h>

// for publishing and subscribing to images in ROS
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

// for subscribing multiple topic with the syncronizer
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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

// for using geometry messages
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

// for using color matrix
#include "colormat.h"

#include "counter_uas_imgproc/msg/bounding_box.hpp"
#include "counter_uas_imgproc/msg/bounding_boxes.hpp"
#include "counter_uas_imgproc/msg/illegal_drone_status.hpp"

#define PI 3.141592
#define R2D 180.0 / PI
#define D2R PI / 180.0

#define DEVICE 0      // GPU id
#define BATCH_SIZE 1  // default batchsize for inference

#define USE_IMAGE_FILES 111
#define USE_VIDEO_FILE 222
#define USE_ROS_TOPIC 333

typedef struct
{
  std::string strLabel;
  int nOrder;
  std::string strColor;
  cv::Scalar scalColor;
} AnnoDB;

typedef struct
{
  int nLabel;
  cv::Rect box;
  cv::Rect roi;
  cv::Point ptCen;
} RawRectInfo;

typedef struct
{
  AnnoDB labelInfo;
  cv::Point ptLt;
  cv::Point ptRb;
  cv::Point ptCen;
  int nLabel;
  int nWidth;
  int nHeight;
  cv::Rect rectBbox;
  float fConfidence;
  float fYoloData[4];
} BboxInfo;

struct RectCenDist
{
  double D;
  RectCenDist(double d) : D(d)
  {
  }
  bool operator()(const RawRectInfo& a, const RawRectInfo& b)
  {
    return sqrt(((a.ptCen.x - b.ptCen.x) * (a.ptCen.x - b.ptCen.x)) +
                ((a.ptCen.y - b.ptCen.y) * (a.ptCen.y - b.ptCen.y))) < D;
  }
};

struct DetectionResult
{
  int frame;
  int id;
  cv::Rect bbox;

  DetectionResult(int frame, int id, cv::Rect bbox) : frame(frame), id(id), bbox(bbox) // , cv::Point ptCen , ptCen(ptCen)
  {
  }
};

#endif // COUNTER_UAS_IMGPROC_GLOBAL_HEADER_H