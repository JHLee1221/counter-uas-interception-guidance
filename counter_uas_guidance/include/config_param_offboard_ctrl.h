#ifndef CONFIG_PARAM_GUIDANCE_OFFBOARD_CTRL_H
#define CONFIG_PARAM_GUIDANCE_OFFBOARD_CTRL_H

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

class ConfigParamOffboardCtrl
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
  explicit ConfigParamOffboardCtrl(std::shared_ptr<rclcpp::Node> node);
  ~ConfigParamOffboardCtrl();

  bool GetRosParams();

  std::string GenLocalTimeStringNormal();
  std::string GenLocalTimeStringFacet();

  int nFrameRate;
  double dmaxSpeed;
  double dKp;
  double dKi;
  double dKd;
  double dAdpGain;
  double dAlpha;
  double dYawAngLmt;
  double dYawRateLmt;
  double dMaxYawRate;
  double dTakeoffAlt;
  double dKpAlt;
  double dAltVelLmt;

  std::string strUasStaTpNmSrc;   // mavros state
  std::string strUasOdomTpNmSrc;  // uas odom
  std::string strTgtOdomTpNmSrc;  // target odom
  std::string strMntAngTpNmSrc;   // mount angle
  std::string strUasSetPtTpNmDst; // Offboard
  std::string strUasArmSrvNmSrc;  // Offboard
  std::string strUasStMdSrvNmSrc; // Offboard
};

#endif // CONFIG_PARAM_GUIDANCE_OFFBOARD_CTRL_H