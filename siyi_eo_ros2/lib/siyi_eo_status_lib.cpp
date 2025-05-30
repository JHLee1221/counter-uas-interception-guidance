#include "siyi_eo_status_lib.h"

using namespace std;
using namespace rclcpp;
using namespace cv;

SiyiEoStatusLib::SiyiEoStatusLib(const ConfigParam& cfg, shared_ptr<rclcpp::Node> node)
  : cfgParam_(cfg)
  , node_(node)
  , running_(false)
  , thread_started_(false)
{
  rclcpp::QoS qos_profile(1);
  qos_profile.reliable();
  qos_profile.keep_last(1);

  RCLCPP_INFO(node_->get_logger(), "Publisher(Image Source): %s", cfgParam_.strRosImgTpNmRawDst.c_str());
  PubRawImgSrc_ = node_->create_publisher<sensor_msgs::msg::Image>(
    cfgParam_.strRosImgTpNmRawDst, qos_profile);

  RCLCPP_INFO(node_->get_logger(), "Publisher(Compressed Image Source): %s", cfgParam_.strRosImgTpNmCompDst.c_str());
  PubCompImgSrc_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
    cfgParam_.strRosImgTpNmCompDst, qos_profile);

  cap_.open(cfgParam_.nCaptureNum);
  if (!cap_.isOpened()) 
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open camera device at %d", cfgParam_.nCaptureNum);
  } 
  else 
  {
    RCLCPP_INFO(node_->get_logger(), "Open camera device at %d", cfgParam_.nCaptureNum);
    if (cfgParam_.nImgSize == 000) 
    {
      cap_.set(CAP_PROP_FRAME_WIDTH, cfgParam_.nVgaWidth);
      cap_.set(CAP_PROP_FRAME_HEIGHT, cfgParam_.nVgaHeight);
      RCLCPP_INFO(node_->get_logger(), "Camera resolution is VGA");
    } 
    else if (cfgParam_.nImgSize == 111) 
    {
      cap_.set(CAP_PROP_FRAME_WIDTH, cfgParam_.nFhdWidth);
      cap_.set(CAP_PROP_FRAME_HEIGHT, cfgParam_.nFhdHeight);
      RCLCPP_INFO(node_->get_logger(), "Camera resolution is FHD");
    } 
    else 
    {
      RCLCPP_ERROR(node_->get_logger(), "Check Params...");
    }
  }
}

SiyiEoStatusLib::~SiyiEoStatusLib()
{
  ShutImageSrc();
}

void SiyiEoStatusLib::MainStatusLoop()
{
  if (!thread_started_) 
  {
    running_ = true;
    publish_thread_ = std::thread(&SiyiEoStatusLib::PubImageSrc, this);
    thread_started_ = true;
    RCLCPP_INFO(node_->get_logger(), "Streaming thread started");
  }
}

void SiyiEoStatusLib::PubImageSrc()
{
  std::vector<int> comp_params = {cv::IMWRITE_JPEG_QUALITY, cfgParam_.nCompQuality};

  while (running_ && rclcpp::ok()) 
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      cap_ >> imgRaw_;
    }  

    if (imgRaw_.empty()) 
    {
      RCLCPP_WARN(node_->get_logger(), "Failed to Read Image from Camera");
      continue;
    }

    imgBridge_.header.stamp = node_->now();
    imgBridge_.header.frame_id = "camera_optical_frame";
    imgBridge_.encoding = "bgr8";
    imgBridge_.image = imgRaw_;
    msg_ = imgBridge_.toImageMsg();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      PubRawImgSrc_->publish(*msg_);
    }

    imencode(".jpg", imgRaw_, buffer, comp_params);
    comp_msg_.header.stamp = node_->now();
    comp_msg_.header.frame_id = "camera_optical_frame";
    comp_msg_.format = "jpeg";
    comp_msg_.data.assign(buffer.begin(), buffer.end());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      PubCompImgSrc_->publish(comp_msg_);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void SiyiEoStatusLib::ShutImageSrc()
{
  if (thread_started_) 
  {
    running_ = false;
    if (publish_thread_.joinable()) 
    {
      publish_thread_.join();
      RCLCPP_INFO(node_->get_logger(), "Streaming thread stopped");
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (cap_.isOpened()) 
    {
      cap_.release();
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Camera is closed");
}