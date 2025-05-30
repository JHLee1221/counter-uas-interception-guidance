#include "config_param_status.h"

using namespace std;
using namespace rclcpp;
using namespace cv;

ConfigParam::ConfigParam(shared_ptr<rclcpp::Node> node)
  : node_(node)
  , nCaptureNum(0)
  , nImgSize(0)
  , nVgaWidth(0)
  , nVgaHeight(0)
  , nFhdWidth(0)
  , nFhdHeight(0)
  , nCompQuality(0)
{
  nVgaWidth = 640;
  nVgaHeight = 480;
  sizeVgaRaw = Size(nVgaWidth, nVgaHeight);

  nFhdWidth = 1920;
  nFhdHeight = 1080;
  sizeFhdRaw = Size(nFhdWidth, nFhdHeight);
  
}

ConfigParam::~ConfigParam()
{
}

// reading ros2 params (public function)
bool ConfigParam::GetRosParams()
{
  return ReadRosParams();
}

// reading ros2 params (private function)
bool ConfigParam::ReadRosParams()
{
  try
  {
    ReadRosParam(node_, "EO_PARAMS.setCaptureNumber", nCaptureNum);
    ReadRosParam(node_, "EO_PARAMS.setImgSize", nImgSize);
    ReadRosParam(node_, "EO_PARAMS.vgaWidth", nVgaWidth);
    ReadRosParam(node_, "EO_PARAMS.vgaHeight", nVgaHeight);
    ReadRosParam(node_, "EO_PARAMS.fhdWidth", nFhdWidth);
    ReadRosParam(node_, "EO_PARAMS.fhdHeight", nFhdHeight);
    ReadRosParam(node_, "EO_PARAMS.setCompQuality", nCompQuality);
    ReadRosParam(node_, "CUI.pubCbInfo.eoImgRaw", strRosImgTpNmRawDst);
    ReadRosParam(node_, "CUI.pubCbInfo.eoImgComp", strRosImgTpNmCompDst);
  }
  catch(RosParamNotFoundException& ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read param at key \"%s\"", ex.key.c_str());
    return false;
  }
  
  return true;
}

void ConfigParam::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, float& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<float>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

void ConfigParam::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, double& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<double>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

void ConfigParam::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, bool& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<bool>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

void ConfigParam::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, int32_t& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<int32_t>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

void ConfigParam::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, string& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<string>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}