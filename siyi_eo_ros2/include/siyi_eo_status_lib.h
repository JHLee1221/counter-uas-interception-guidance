#ifndef SIYI_EO_STATUS_LIB_H
#define SIYI_EO_STATUS_LIB_H

#include "global_header.h"
#include "config_param_status.h"

class SiyiEoStatusLib
{
public:
  SiyiEoStatusLib(const ConfigParam& cfg, std::shared_ptr<rclcpp::Node> node);
  ~SiyiEoStatusLib();

  void MainStatusLoop();
  void ShutImageSrc();

private:
  // config parameters
  ConfigParam cfgParam_;
  
  // image transport with cv bridge
  cv::VideoCapture cap_;
  cv::Mat imgRaw_;
  cv_bridge::CvImage imgBridge_;

  // ros2 node & msg
  std::shared_ptr<rclcpp::Node> node_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  sensor_msgs::msg::CompressedImage comp_msg_;
  std::vector<int> comp_params;

  void PubImageSrc();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr PubRawImgSrc_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr PubCompImgSrc_;

  std::atomic<bool> running_;
  std::vector<uchar> buffer;
  std::atomic<bool> thread_started_;
  std::thread publish_thread_;
  std::mutex mutex_;
};

#endif // SIYI_EO_STATUS_LIB_H