#include "config_param_detection.h"

using namespace std;
using namespace rclcpp;

ConfigParamDetection::ConfigParamDetection(shared_ptr<rclcpp::Node> node)
  : node_(node)
  , nFrameRate(0)
  , nPlayMode(0)
  , bUseSaveInfo(false)
  , bUseSaveImgVidOnly(false)
  , bUseDebugMode(false)
  , fNmsThresh(0.0)
  , fConfThresh(0.0)
  , dSaveResImgDt(0.0)
{
  nVgaWidth = 640;
  nVgaHeight = 480;
  sizeVgaRaw = Size(nVgaWidth, nVgaHeight);

  nFhdWidth = 1920;
  nFhdHeight = 1080;
  sizeFhdRaw = Size(nFhdWidth, nFhdHeight);
}

ConfigParamDetection::~ConfigParamDetection()
{
}

bool ConfigParamDetection::GetRosParams()
{
  return ReadRosParams();
}

bool ConfigParamDetection::ReadRosParams()
{
  try
  {
    // generation information
    strHomeName_ = getenv("HOME");

    // reading ros2 param via yaml file, general option
    ReadRosParam(node_, "CUI.frameRate", nFrameRate);
    ReadRosParam(node_, "CUI.saveResImgDt", dSaveResImgDt);
    ReadRosParam(node_, "CUI.engineFile", strFileNameDetecionEngine);
    RCLCPP_INFO(node_->get_logger(), "Load YOLO engine file path: %s", strFileNameDetecionEngine.c_str());
    ReadRosParam(node_, "CUI.useDebugMode", bUseDebugMode);
    ReadRosParam(node_, "CUI.inferPlayMode.mode", nPlayMode);
    ReadRosParam(node_, "CUI.inferPlayMode.enableSaveRes", bUseSaveInfo);
    ReadRosParam(node_, "CUI.inferPlayMode.enableSaveImgVidOnly", bUseSaveImgVidOnly);
    ReadRosParam(node_, "CUI.inferPlayMode.saveFileName", strSaveFileName);
    strFileNameDetecionEngine = strHomeName_ + strFileNameDetecionEngine;
    
    // reading ros2 param via yaml file, topic option
    ReadRosParam(node_, "CUI.inferPlayMode.topic.imgDstTopicName", strRosImgTpNmDst);
    ReadRosParam(node_, "CUI.inferPlayMode.topic.compDstTopicName", strRosImgTpNmCompDst);
    ReadRosParam(node_, "CUI.inferPlayMode.topic.sourceTopicName", strRosImgTpNmSrc);
    ReadRosParam(node_, "CUI.inferPlayMode.topic.saveFrameSrcPath", strRosImgFrameSrcFolderInfo);
    ReadRosParam(node_, "CUI.inferPlayMode.topic.saveFrameDstPath", strRosImgFrameDstFolderInfo);
    ReadRosParam(node_, "CUI.inferPlayMode.topic.saveLabelDstPath", strRosImgLabelDstFolderInfo);
    strRosImgFrameSrcFolderInfo = strHomeName_ + strRosImgFrameSrcFolderInfo;
    strRosImgFrameDstFolderInfo = strHomeName_ + strRosImgFrameDstFolderInfo;
    strRosImgLabelDstFolderInfo = strHomeName_ + strRosImgLabelDstFolderInfo;

    // reading ros2 param via yaml file, video option
    ReadRosParam(node_, "CUI.inferPlayMode.video.folderPath", strVidFolderPath);
    ReadRosParam(node_, "CUI.inferPlayMode.video.fileName", strVidFileName);
    ReadRosParam(node_, "CUI.inferPlayMode.video.imgRawTopicName", strVidImgRawTpNm);
    ReadRosParam(node_, "CUI.inferPlayMode.video.saveImgSrcPath", strVidImgSrcFolderInfo);
    ReadRosParam(node_, "CUI.inferPlayMode.video.saveImgDstPath", strVidImgDstFolderInfo);
    ReadRosParam(node_, "CUI.inferPlayMode.video.saveLabelDstPath", strVidLabelDstFolderInfo);
    ReadRosParam(node_, "CUI.inferPlayMode.video.saveVidDstPath", strVidSaveDstFileInfo);
    strVidSrcFileInfo = strHomeName_ + strVidFolderPath + strVidFileName;
    strVidImgSrcFolderInfo = strHomeName_ + strVidImgSrcFolderInfo;
    strVidImgDstFolderInfo = strHomeName_ + strVidImgDstFolderInfo;
    strVidLabelDstFolderInfo = strHomeName_ + strVidLabelDstFolderInfo;
    strVidSaveDstFileInfo = strHomeName_ + strVidSaveDstFileInfo;

    // reading ros2 param via yaml file, image option
    ReadRosParam(node_, "CUI.inferPlayMode.image.srcFolderPath", strImgSrcFolderInfo);
    ReadRosParam(node_, "CUI.inferPlayMode.image.dstFolderPath", strImgImgDstFolderInfo);
    ReadRosParam(node_, "CUI.inferPlayMode.image.labelFolderPath", strImgLabelDstFolderInfo);
    strImgSrcFolderInfo = strHomeName_ + strImgSrcFolderInfo;
    strImgImgDstFolderInfo = strHomeName_ + strImgImgDstFolderInfo;
    strImgLabelDstFolderInfo = strHomeName_ + strImgLabelDstFolderInfo;

    // reading ros2 param via yaml file, threshold option
    ReadRosParam(node_, "CUI.threshold.nmsThVal", fNmsThresh);
    ReadRosParam(node_, "CUI.threshold.confVal", fConfThresh);

    // reading parameters, for generating the label information
    vecAntiDroneLabelInfo.clear();
    ReadRosParam(node_, "LabelKariAntiDroneDB.drone.name", drone_.strLabel);
    ReadRosParam(node_, "LabelKariAntiDroneDB.drone.order", drone_.nOrder);
    ReadRosParam(node_, "LabelKariAntiDroneDB.drone.color", drone_.strColor);
    drone_.scalColor = colorStat_.GenColorInfo(drone_.strColor);
    vecAntiDroneLabelInfo.push_back(drone_);

    ReadRosParam(node_, "LabelKariAntiDroneDB.bird.name", bird_.strLabel);
    ReadRosParam(node_, "LabelKariAntiDroneDB.bird.order", bird_.nOrder);
    ReadRosParam(node_, "LabelKariAntiDroneDB.bird.color", bird_.strColor);
    bird_.scalColor = colorStat_.GenColorInfo(bird_.strColor);
    vecAntiDroneLabelInfo.push_back(bird_);

    ReadRosParam(node_, "LabelKariAntiDroneDB.djiPhantom.name", djiPhantom_.strLabel);
    ReadRosParam(node_, "LabelKariAntiDroneDB.djiPhantom.order", djiPhantom_.nOrder);
    ReadRosParam(node_, "LabelKariAntiDroneDB.djiPhantom.color", djiPhantom_.strColor);
    djiPhantom_.scalColor = colorStat_.GenColorInfo(djiPhantom_.strColor);
    vecAntiDroneLabelInfo.push_back(djiPhantom_);

    ReadRosParam(node_, "LabelKariAntiDroneDB.djiMavick.name", djiMavick_.strLabel);
    ReadRosParam(node_, "LabelKariAntiDroneDB.djiMavick.order", djiMavick_.nOrder);
    ReadRosParam(node_, "LabelKariAntiDroneDB.djiMavick.color", djiMavick_.strColor);
    djiMavick_.scalColor = colorStat_.GenColorInfo(djiMavick_.strColor);
    vecAntiDroneLabelInfo.push_back(djiMavick_);

    ReadRosParam(node_, "LabelKariAntiDroneDB.djiM600.name", djiM600_.strLabel);
    ReadRosParam(node_, "LabelKariAntiDroneDB.djiM600.order", djiM600_.nOrder);
    ReadRosParam(node_, "LabelKariAntiDroneDB.djiM600.color", djiM600_.strColor);
    djiM600_.scalColor = colorStat_.GenColorInfo(djiM600_.strColor);
    vecAntiDroneLabelInfo.push_back(djiM600_);

    ReadRosParam(node_, "LabelKariAntiDroneDB.djiMatrice.name", djiMatrice_.strLabel);
    ReadRosParam(node_, "LabelKariAntiDroneDB.djiMatrice.order", djiMatrice_.nOrder);
    ReadRosParam(node_, "LabelKariAntiDroneDB.djiMatrice.color", djiMatrice_.strColor);
    djiMatrice_.scalColor = colorStat_.GenColorInfo(djiMatrice_.strColor);
    vecAntiDroneLabelInfo.push_back(djiMatrice_);

    ReadRosParam(node_, "LabelKariAntiDroneDB.djiInspire.name", djiInspire_.strLabel);
    ReadRosParam(node_, "LabelKariAntiDroneDB.djiInspire.order", djiInspire_.nOrder);
    ReadRosParam(node_, "LabelKariAntiDroneDB.djiInspire.color", djiInspire_.strColor);
    djiInspire_.scalColor = colorStat_.GenColorInfo(djiInspire_.strColor);
    vecAntiDroneLabelInfo.push_back(djiInspire_);

    ReadRosParam(node_, "LabelKariAntiDroneDB.arDrone.name", arDrone_.strLabel);
    ReadRosParam(node_, "LabelKariAntiDroneDB.arDrone.order", arDrone_.nOrder);
    ReadRosParam(node_, "LabelKariAntiDroneDB.arDrone.color", arDrone_.strColor);
    arDrone_.scalColor = colorStat_.GenColorInfo(arDrone_.strColor);
    vecAntiDroneLabelInfo.push_back(arDrone_);

    ReadRosParam(node_, "LabelKariAntiDroneDB.djiAgras.name", djiAgras_.strLabel);
    ReadRosParam(node_, "LabelKariAntiDroneDB.djiAgras.order", djiAgras_.nOrder);
    ReadRosParam(node_, "LabelKariAntiDroneDB.djiAgras.color", djiAgras_.strColor);
    djiAgras_.scalColor = colorStat_.GenColorInfo(djiAgras_.strColor);
    vecAntiDroneLabelInfo.push_back(djiAgras_);

    ReadRosParam(node_, "LabelKariAntiDroneDB.background.name", bgAntiDrone_.strLabel);
    ReadRosParam(node_, "LabelKariAntiDroneDB.background.order", bgAntiDrone_.nOrder);
    ReadRosParam(node_, "LabelKariAntiDroneDB.background.color", bgAntiDrone_.strColor);
    bgAntiDrone_.scalColor = colorStat_.GenColorInfo(bgAntiDrone_.strColor);
    vecAntiDroneLabelInfo.push_back(bgAntiDrone_);
  }
  catch (RosParamNotFoundException& ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read param at key \"%s\"", ex.key.c_str());
    return false;
  }
  
  return true;
}

// generating local time to string with facet
string ConfigParamDetection::GenLocalTimeStringFacet()
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
string ConfigParamDetection::GenLocalTimeStringNormal()
{
  std::stringstream ss;
  boost::local_time::local_date_time currentTime(boost::posix_time::second_clock::local_time(),
                                                 boost::local_time::time_zone_ptr());
  ss << currentTime.local_time();

  return ss.str();
}

void ConfigParamDetection::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, float& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<float>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

void ConfigParamDetection::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, double& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<double>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

void ConfigParamDetection::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, bool& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<bool>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

void ConfigParamDetection::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, int32_t& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<int32_t>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}

void ConfigParamDetection::ReadRosParam(shared_ptr<rclcpp::Node>& node, const string& key, string& val)
{
  // predeclare parameters with defaults to avoid exceptions
  node->declare_parameter<string>(key);

  // check existence and get value
  if (!node->get_parameter(key, val))
  {
    throw RosParamNotFoundException(key); // Throw if missing
  }
}