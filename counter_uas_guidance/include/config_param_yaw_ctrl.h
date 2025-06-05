#ifndef CONFIG_PARAM_GUIDANCE_YAW_CTRL_H
#define CONFIG_PARAM_GUIDANCE_YAW_CTRL_H

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

class ConfigParamYawCtrl
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
  explicit ConfigParamYawCtrl(std::shared_ptr<rclcpp::Node> node);
  ~ConfigParamYawCtrl();

  bool GetRosParams();

  std::string GenLocalTimeStringNormal();
  std::string GenLocalTimeStringFacet();

  int nFrameRate;
  float fYawErr;
  float fYawErrI;
  float fYawErrD;
  float fPrevYawErr;
  float fRawYawRate;
  float fFltYawRate;
  float fmaxSpeed;
  float fKp;
  float fKi;
  float fKd;
  float fAdpGain;
  bool bGuidanceMode;

  std::string strUasStaTpNmSrc;   // mavros state
  std::string strUasOdomTpNmSrc;  // uas odom
  std::string strTgtOdomTpNmSrc;  // target odom
  std::string strMntAngTpNmSrc;   // mount angle
  std::string strUasSetPtTpNmDst; // Offboard
  std::string strUasArmSrvNmSrc;  // Offboard
  std::string strUasStMdSrcNmSrc; // Offboard
};

#endif // CONFIG_PARAM_GUIDANCE_YAW_CTRL_H