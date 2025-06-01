#ifndef COUNTER_UAS_IMGPROC_COUNTER_UAS_YOLOV8P2_INFER_LIB_H
#define COUNTER_UAS_IMGPROC_COUNTER_UAS_YOLOV8P2_INFER_LIB_H

#include "global_header.h"
#include "config_param_detection.h"
#include "misc_calc_lib.h"
#include "misc_func.h"

#include "yolov8_p2_infer_engine/yolov8_p2_infer_engine.h"

class CounterUasYoloV8P2InferLib
{
public:
  CounterUasYoloV8P2InferLib(const ConfigParamDetection& cfg, std::shared_ptr<rclcpp::Node> node);
  ~CounterUasYoloV8P2InferLib();

  void MainInferLoop(int nMode);

private:
  ConfigParamDetection cfgParam_;
  std::shared_ptr<rclcpp::Node> node_;
  MiscCalc miscCalc_;
  MiscFunction miscFunc_;

  YoloV8P2InferEngine yoloInfEng_;

  cv_bridge::CvImagePtr cvPtrImgSrc_;
  cv::Mat imgRaw_;
  double dTimeCounter_;
  cv::VideoWriter vidWriter_;
  bool bStartCamCallBack_;

  void CbRawImgData(const sensor_msgs::msg::Image::ConstSharedPtr& msgRaw);
  void GenRosImgTopicLoop(cv::Mat imgSrc);
  void GenVidLoop(std::string strVidFile);
  void GenImgLoop(std::string strImgFolder);
  void SaveInfResImgLabel(cv::Mat imgRes, cv::Mat imgSrc, std::vector<BboxInfo> vecInfo, std::string strImgDstInfo,
                          std::string strLabelDstInfo, std::string strImgOnlyInfo);
  void PubInferBboxes(std::vector<BboxInfo> vecInput, cv::Mat imgRaw, cv::Mat imgDst);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subRawImg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubVidSrcImg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubDetResImg_;
  rclcpp::Publisher<counter_uas_imgproc::msg::BoundingBoxes>::SharedPtr pubDetResBboxes_;
};

#endif // COUNTER_UAS_IMGPROC_COUNTER_UAS_YOLOV8P2_INFER_LIB_H