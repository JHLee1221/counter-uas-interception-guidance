#ifndef YOLOV5_TRT_INFER_MISC_CALC_LIB_H
#define YOLOV5_TRT_INFER_MISC_CALC_LIB_H

#include "global_header.h"
#include "config_param.h"

using namespace std;
using namespace rclcpp;
using namespace cv;

class MiscCalc
{
public:
  MiscCalc(const ConfigParam& cfg, std::shared_ptr<rclcpp::Node> node);
  ~MiscCalc();

  bool GetSizeCalcFlag();
  bool GetVidFlag();
  void DrawVectorLabeledRect(Mat imgSrc, vector<BboxInfo> vecInfo);
  void DrawSingleLabeledRect(Mat imgSrc, BboxInfo bboxInfo);
  void SaveImgRaw(Mat imgSrc, string path);
  void SaveImgRes(Mat imgSrc, string pathImg, string pathLabel, vector<BboxInfo> vecInfo, bool bFeatUse);
  Size GenImgSize(Mat imgSrc);
  bool GenSizeCalcFlag(int nSize, int nTotal);
  string GetConfString(float fInput, int nNumFlot);
  int Saturation(int val, int min, int max);

  bool bSizeCalcFlag;
  bool bVidStatus;

private:
  ConfigParam cfgParam_;
  ColorStatus colorStat_;
  std::shared_ptr<rclcpp::Node> node_;
};

#endif  // YOLOV5_TRT_INFER_MISC_CALC_LIB_H
