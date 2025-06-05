#include "config_param_yaw_ctrl.h"

using namespace std;
using namespace rclcpp;

ConfigParamYawCtrl::ConfigParamYawCtrl(shared_ptr<rclcpp::Node> node)
  : node_(node) 
  , nFrameRate(0)
  , fYawErr(0.0)
  , fYawErrI(0.0)
  , fYawErrD(0.0)
  , fPrevYawErr(0.0)
  , fRawYawRate(0.0)
  , fFltYawRate(0.0)
  , fmxSpeed(0.0)
  , fKp(0.0)
  , fKi(0.0)
  , fKd(0.0)
  , fAdpGain(0.0)
  , bGuidanceMode(false)
{
}

ConfigParamYawCtrl::~ConfigParamYawCtrl()
{
}

bool ConfigParamYawCtrl::GetRosParams()
{
  return ReadRosParams();
}

bool ConfigParamYawCtrl::ReadRosParams()
{
  try
  {
    // Reading ros2 param via yaml file, general options
    ReadRosParam(node_, "CUI.frameRate", nFrameRate);
    ReadRosParam(node_, "CUI.guidanceMode", bGuidanceMode);
    ReadRosParam(node_, "CUI.yawCtrl.maxSpeed", fmaxSpeed);
    ReadRosParam(node_, "CUI.yawCtrl.error.yaw", fYawErr);
    ReadRosParam(node_, "CUI.yawCtrl.error.yawIntg", fYawErrI);
    ReadRosParam(node_, "CUI.yawCtrl.error.yawDiff", fYawErrD);
    ReadRosParam(node_, "CUI.yawCtrl.error.yawPrev", fPrevYawErr);
    ReadRosParam(node_, "CUI.yawCtrl.rate.raw", fRawYawRate);
    ReadRosParam(node_, "CUI.yawCtrl.rate.filtering", fFltYawRate);
    ReadRosParam(node_, "CUI.yawCtrl.gain.Kp", fKp);
    ReadRosParam(node_, "CUI.yawCtrl.gain.Ki", fKi);
    ReadRosParam(node_, "CUI.yawCtrl.gain.Kd", fKd);
    ReadRosParam(node_, "CUI.yawCtrl.gain.adaptive", fAdpGain);
    ReadRosParam(node_, "CUI.")
    /* code */
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
}