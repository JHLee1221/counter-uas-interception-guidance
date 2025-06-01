#ifndef CONFIG_PARAM_DETECTION_H
#define CONFIG_PARAM_DETECTION_H

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

class ConfigParamDetection
{
private:
  std::shared_ptr<rclcpp::Node> node_;
  ColorStatus colorStat_;

  bool ReadRosParams();
  void ReadRosParam(std::shared_ptr<rclcpp::Node>& node, const std::string& key, float& val);
  void ReadRosParam(std::shared_ptr<rclcpp::Node>& node, const std::string& key, double& val);
  void ReadRosParam(std::shared_ptr<rclcpp::Node>& node, const std::string& key, bool& val);
  void ReadRosParam(std::shared_ptr<rclcpp::Node>& node, const std::string& key, int32_t& val);
  void ReadRosParam(std::shared_ptr<rclcpp::Node>& node, const std::string& key, std::string& val);

  std::string strHomeName_;

  AnnoDB drone_;
  AnnoDB bird_;
  AnnoDB djiPhantom_;
  AnnoDB djiMavick_;
  AnnoDB djiM600_;
  AnnoDB djiMatrice_;
  AnnoDB djiInspire_;
  AnnoDB arDrone_;
  AnnoDB djiAgras_;
  AnnoDB bgAntiDrone_;

public:
  explicit ConfigParamDetection(std::shared_ptr<rclcpp::Node> node);
  ~ConfigParamDetection();

  bool GetRosParams();
  std::string GenLocalTimeStringNormal();
  std::string GenLocalTimeStringFacet();

  int nVgaWidth;
  int nVgaHeight;
  Size sizeVgaRaw;

  int nFhdWidth;
  int nFhdHeight;
  Size sizeFhdRaw;

  int nFrameRate;
  int nPlayMode;
  bool bUseSaveInfo;
  bool bUseSaveImgVidOnly;

  bool bUseDebugMode;
  float fNmsThresh;
  float fConfThresh;

  double dSaveResImgDt;

  std::string strFileNameDetecionEngine;
  std::string strSaveFileName;

  std::string strRosImgTpNmSrc;
  std::string strRosImgTpNmDst;
  std::string strRosImgTpNmCompDst;
  std::string strRosImgFrameSrcFolderInfo;
  std::string strRosImgFrameDstFolderInfo;
  std::string strRosImgLabelDstFolderInfo;
  std::string strVidFolderPath;
  std::string strVidFileName;
  std::string strVidSrcFileInfo;
  std::string strVidImgRawTpNm;
  std::string strVidImgSrcFolderInfo;
  std::string strVidImgDstFolderInfo;
  std::string strVidLabelDstFolderInfo;
  std::string strVidSaveDstFileInfo;
  std::string strImgSrcFolderInfo;
  std::string strImgImgDstFolderInfo;
  std::string strImgLabelDstFolderInfo;

  std::vector<AnnoDB> vecAntiDroneLabelInfo;
};

#endif // CONFIG_PARAM_DETECTION_H