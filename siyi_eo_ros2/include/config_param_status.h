#ifndef CONFIG_PARAM_STATUS_H
#define CONFIG_PARAM_STATUS_H

#include "global_header.h"

class RosParamNotFoundException : public std::exception
{
public:
  std::string key;
  std::string error_msg;

  explicit RosParamNotFoundException(const std::string& key_)
    : key(key_), error_msg("Failed to read param at key " + key_) {}

  virtual const char* what() const noexcept override
  {
    return error_msg.c_str();
  }
};

class ConfigParam
{
private:
  std::shared_ptr<rclcpp::Node> node_;

  bool ReadRosParams();
  void ReadRosParam(std::shared_ptr<rclcpp::Node>& node, const std::string& key, float& val);
  void ReadRosParam(std::shared_ptr<rclcpp::Node>& node, const std::string& key, double& val);
  void ReadRosParam(std::shared_ptr<rclcpp::Node>& node, const std::string& key, bool& val);
  void ReadRosParam(std::shared_ptr<rclcpp::Node>& node, const std::string& key, int32_t& val);
  void ReadRosParam(std::shared_ptr<rclcpp::Node>& node, const std::string& key, std::string& val);

public:
  explicit ConfigParam(std::shared_ptr<rclcpp::Node> node);
  ~ConfigParam();

  bool GetRosParams();

  int nCaptureNum;
  int nImgSize;

  int nVgaWidth;
  int nVgaHeight;
  cv::Size sizeVgaRaw;

  int nFhdWidth;
  int nFhdHeight;
  cv::Size sizeFhdRaw;

  int nCompQuality;

  std::string strRosImgTpNmRawDst;
  std::string strRosImgTpNmCompDst;
  // std::string strRosMountTpNmCmdDst;

};

#endif // SIYI_EO_ROS2_CONFIG_PARAM_H