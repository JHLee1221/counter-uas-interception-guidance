#include "config_param_mount_ctrl.h"

using namespace std;
using namespace cv;
using namespace rclcpp;

ConfigParamMountCtrl::ConfigParamMountCtrl(shared_ptr<rclcpp::Node> node)
  : node_(node)
  , nVgaWidth(0)
  , nVgaHeight(0)
  , nFhdWidth(0)
  , nFhdHeight(0)
  , nFrameRate(0)
  , fPgain(0.0f)
  , fIgain(0.0f)
  , fDgain(0.0f)
  , fAlpha(0.0f)
  , fHfovDeg(0.0f)
{
  nVgaWidth = 640;
  nVgaHeight = 480;
  sigeVgaRaw = Size(nVgaWidth, nVgaHeight);
  
  nFhdWidth = 1920;
  nFhdHeight = 1080;
  sigeFhdRaw = Size(nFhdWidth, nFhdHeight);
}

ConfigParamMountCtrl::~ConfigParamMountCtrl()
{
}

bool ConfigParamMountCtrl::GetRosParams()
{
  return ReadRosParams();
}

bool ConfigParamMountCtrl::ReadRosParams()
{
  try 
  {
    // Generation information
    strHomeName_ = getenv("HOME");

    // Reading ros2 param via yaml file, generatl options
    ReadRosParam(node_, "CUI.frameRate", nFrameRate);
    ReadRosParam(node_, "CUI.hFovDeg", fHfovDeg);

    // Reading ros2 param via yaml file, pid ctrl options
    ReadRosParam(node_, "CUI.mountCtrl.pidCtrl.pGain", fPgain);
    ReadRosParam(node_, "CUI.mountCtrl.pidCtrl.iGain", fIgain);
    ReadRosParam(node_, "CUI.mountCtrl.pidCtrl.dGain", fDgain);
    ReadRosParam(node_, "CUI.mountCtrl.pidCtrl.alpha", fAlpha);

    // Reading ros2 param via yaml file, topic options
    ReadRosParam(node_, "CUI.mountCtrl.topic.imgSrcTopicName", strRosImgTpNmSrc);
    ReadRosParam(node_, "CUI.mountCtrl.topic.trkResTopiName", strTrkTpNmVisDetArrRes);
    ReadRosParam(node_, "CUI.mountCtrl.topic.mountCtrlCmdTopicName", strMountCtrlCmdDst);

  }
  catch (RosParamNotFoundException& ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read param at key \"%s\"", ex.key.c_str());
    return false;
  }

  return true;
}

// Generating local time to string with facet
string ConfigParamMountCtrl::GenLocalTimeStringFacet()
{
  stringstream ss;
  boost::local_time::local_date_time currentTime(boost::posix_time::microsec_clock::local_time(),
                                                 boost::local_time::time_zone_ptr());
  auto facet = new boost::posix_time::time_facet("%Y_%m_%d_%H_%M_%s");
  ss.imbue(locale(locale::classic(), facet));
  ss << currentTime.local_time();

  return ss.str();
}

// Generating local time to string without facet
string ConfigParamMountCtrl::GenLocalTimeStringNormal()
{
  stringstream ss;
  boost::local_time::local_date_time currentTime(boost::posix_time::second_clock::local_time(),
                                                 boost::local_time::time_zone_ptr());
  ss << currentTime.local_time();

  return ss.str();
}
// Reading ros2 param with float type
void ConfigParamMountCtrl::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, float& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<float>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

// Reading ros2 param with double type
void ConfigParamMountCtrl::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, double& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<double>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

// Reading ros2 param with bool type
void ConfigParamMountCtrl::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, bool& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<bool>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

// Reading ros2 param with int32_t type
void ConfigParamMountCtrl::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, int32_t& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<int32_t>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

// Reading ros2 param with string type
void ConfigParamMountCtrl::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, string& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<string>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}