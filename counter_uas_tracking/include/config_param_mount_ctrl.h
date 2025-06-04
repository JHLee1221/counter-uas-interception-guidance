#ifndef CONFIG_PARAM_MOUNT_CTRL_H
#define CONFIG_PARAM_MOUNT_CTRL_H

#include "global_header.h"

class RosParamNotFoundException : public std::exception
{
public:
  std::string key;
  std::string error_msg;

  explicit RosParamNotFoundException(const std::std::string& key_)
    : key(key_), error_msg("Failed to read param at key " + key_) {}

  virtual const char* what() const noexcept override
  {
    return error_msg.c_str();
  }
};

class ConfigParamMountCtrl
{
public:
  explicit ConfigParamMountCtrl(std::shared_ptr<rclcpp::Node> node);
  ~ConfigParamMountCtrl();

  bool GetRosParam();

  std::string GenLocalTimeStringNormal();
  std::string GenLocalTimeStringFacet();

  int nVgaWidth;
  int nVgaHeight;
  cv::Size sigeVgaRaw;

  int nFhdWidth;
  int nFhdHeight;
  cv::Size sigeFhdRaw;

  int nFrameRate;

  std::string strRosImgTpNmSrc;
  std::string strTrkTpNmVisDetArrRes;
  std::string strMountCtrlCmdDst;

  float fPgain;
  float fIgain; 
  float fDgain;
  float fAlpha;
  float fHfovDeg;

private:
  std::shared_ptr<rclcpp::Node> node_;

  bool ReadRosParam();
  void ReadRosParam(std::shared_ptr<rclcpp::Node>& node, const std::string& key, float& val);
  void ReadRosParam(std::shared_ptr<rclcpp::Node>& node, const std::string& key, double& val);
  void ReadRosParam(std::shared_ptr<rclcpp::Node>& node, const std::string& key, bool& val);
  void ReadRosParam(std::shared_ptr<rclcpp::Node>& node, const std::string& key, int32_t& val);
  void ReadRosParam(std::shared_ptr<rclcpp::Node>& node, const std::string& key, std::string& val);

};

#endif // CONFIG_PARAM_MOUNT_CTRL_H