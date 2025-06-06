#include "config_param_offboard_ctrl.h"

using namespace std;
using namespace rclcpp;


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


ConfigParamOffboardCtrl::ConfigParamOffboardCtrl(shared_ptr<rclcpp::Node> node)
  : node_(node) 
  , nFrameRate(0)
  , dmxSpeed(0.0)
  , dKp(0.0)
  , dKi(0.0)
  , dKd(0.0)
  , dAdpGain(0.0)
  , dYawAngLmt(0.0)
  , dYawRateLmt(0.0)
  , dMaxYawRate(0.0)
  , dTakeoffAlt(0.0)
  , dKpAlt(0.0)
  , dAltVelLmt(0.0)
{
}

ConfigParamOffboardCtrl::~ConfigParamOffboardCtrl()
{
}

bool ConfigParamOffboardCtrl::GetRosParams()
{
  return ReadRosParams();
}

bool ConfigParamOffboardCtrl::ReadRosParams()
{
  try
  {
    // Reading ros2 param via yaml file, general options
    ReadRosParam(node_, "CUI.offboard.frameRate", nFrameRate);
    ReadRosParam(node_, "CUI.offboard.maxSpeed", dmaxSpeed);
    ReadRosParam(node_, "CUI.offboard.maxYawRate", dMaxYawRate);
    ReadRosParam(node_, "CUI.offboard.altVelLmt", dAltVelLmt);

    // Reading ros2 param via yaml file, takeoff options
    ReadRosParam(node_, "CUI.offboard.takeoff.altitude", dTakeoffAlt);
    ReadRosParam(node_, "CUI.offboard.takeoff.kpGain", dKpAlt);

    // Reading ros2 param via yaml file, yaw rate control options
    ReadRosParam(node_, "CUI.yawCtrl.gain.Kp", dKp);
    ReadRosParam(node_, "CUI.yawCtrl.gain.Ki", dKi);
    ReadRosParam(node_, "CUI.yawCtrl.gain.Kd", dKd);
    ReadRosParam(node_, "CUI.yawCtrl.gain.adaptive", dAdpGain);
    ReadRosParam(node_, "CUI.yawCtrl.filter.alpha", dAlpha);
    ReadRosParam(node_, "CUI.yawCtrl.limits.yawAngle", dYawAngLmt);
    ReadRosParam(node_, "CUI.yawCtrl.limits.yawRate", dYawRateLmt);

    // Reading ros2 param via yaml file, topic options
    ReadRosParam(node_, "CUI.offboard.topic.uasState", strUasStaTpNmSrc);
    ReadRosParam(node_, "CUI.offboard.topic.uasOdom", strUasOdomTpNmSrc);
    ReadRosParam(node_, "CUI.offboard.topic.tgtOdom", strTgtOdomTpNmSrc);
    ReadRosParam(node_, "CUI.offboard.topic.mountAng", strMntAngTpNmSrc);
    ReadRosParam(node_, "CUI.offboard.topic.uasSetPt", strUasSetPtTpNmDst);

    // Reading ros2 param via yaml file, service options
    ReadRosParam(node_, "CUI.offboard.service.arming", strUasArmSrvNmSrc);
    ReadRosParam(node_, "CUI.offboard.service.setMode", strUasStMdSrvNmSrc);
  }
  catch (RosParamNotFoundException& ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read param at key \"%s\"", ex.key.c_str());
    return false;
  }
  return true; 
}

// generating local time to string with facet
string ConfigParamOffboardCtrl::GenLocalTimeStringFacet()
{
  std::stringstream ss;
  boost::local_time::local_date_time currentTime(boost::posix_time::microsec_clock::local_time(),
                                                 boost::local_time::time_zone_ptr());
  auto facet = new boost::posix_time::time_facet("%Y_%m_%d_%H_%M_%s");
  ss.imbue(std::locale(std::locale::classic(), facet));
  ss << currentTime.local_time();

  return ss.str();
}

// generating local time to string without facet
string ConfigParamOffboardCtrl::GenLocalTimeStringNormal()
{
  std::stringstream ss;
  boost::local_time::local_date_time currentTime(boost::posix_time::second_clock::local_time(),
                                                 boost::local_time::time_zone_ptr());
  ss << currentTime.local_time();

  return ss.str();
}

void ConfigParamOffboardCtrl::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, float& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<float>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

void ConfigParamOffboardCtrl::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, double& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<double>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

void ConfigParamOffboardCtrl::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, bool& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<bool>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

void ConfigParamOffboardCtrl::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, int32_t& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<int32_t>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

void ConfigParamOffboardCtrl::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, string& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<string>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}