#include "counter_uas_yolov8p2_infer_lib.h"

using namespace std;
using namespace rclcpp;
using namespace cv;

CounterUasYoloV8P2InferLib::CounterUasYoloV8P2InferLib(const ConfigParamDetection& cfg, shared_ptr<rclcpp::Node> node)
  : cfgParam_(cfg)
  , node_(node)
  , miscCalc_(cfg, node)
  , miscFunc_(cfg, node)
  , yoloInfEng_(cfg, node)
{
  // Deserializing the tensorrt engine file
  if (!yoloInfEng_.CalcTRTEngineInfo(cfgParam_.strFileNameDetecionEngine))
  {
    RCLCPP_ERROR(node_->get_logger(), "CounterUasYoloV8P2InferLib::InitClass::Error in deserializing the tensorrt engine file..");
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "CounterUasYoloV8P2InferLib::InitClass::TensorRT engine file is successfully deserialized..");
  }
    
  // Preparing buffer for inference
  yoloInfEng_.PrepareBuffer(yoloInfEng_.engine);
  RCLCPP_INFO(node_->get_logger(), "CounterUasYoloV8P2InferLib::InitClass::TensorRT engine has prepare memory buffer successfully..");

  // for debugging
  // ImgSize(width,height):(%d,%d)-->imgRaw.cols, imgRaw.rows
  // checking the opencv version, it should 4.2.0 or above because the cuda-assisted opencv-dnn module
  RCLCPP_INFO(node_->get_logger(), "CounterUasYoloV8P2InferLib::InitClass::OpenCV_version:%s", CV_VERSION);

  //Generating image subscriber with compressed option
  RCLCPP_INFO(node_->get_logger(), "Subscriber(rawImage, from EO sensor): %s", cfgParam_.strRosImgTpNmSrc.c_str());
  subRawImg_ = node_->create_subscription<sensor_msgs::msg::Image>(
    cfgParam_.strRosImgTpNmSrc, rclcpp::QoS(1),
    bind(&CounterUasYoloV8P2InferLib::CbRawImgData, this, placeholders::_1));

  // Generating publisher for the video source image
  RCLCPP_INFO(node_->get_logger(), "Publisher(video raw image only, counter-uas video src): %s", cfgParam_.strVidImgRawTpNm.c_str());
  pubVidSrcImg_ = node_->create_publisher<sensor_msgs::msg::Image>(cfgParam_.strVidImgRawTpNm, 1);

  // generating publisher for the inferred result image, detection result
  RCLCPP_INFO(node_->get_logger(), "Publisher(inferred result only, counter-uas, result): %s", cfgParam_.strRosImgTpNmDst.c_str());
  pubDetResImg_ = node_->create_publisher<sensor_msgs::msg::Image>(cfgParam_.strRosImgTpNmDst, 1);

  // generating publisher for the inferred bboxes, detection result
  RCLCPP_INFO(node_->get_logger(), "Publisher(inferred bboxes with images,counter-uas, result): %s",
           cfgParam_.strRosImgTpNmCompDst.c_str());
  pubDetResBboxes_ = node_->create_publisher<counter_uas_imgproc::msg::BoundingBoxes>(cfgParam_.strRosImgTpNmCompDst, 1);

  // initializing variables
  dTimeCounter_ = 0.0;
  bStartCamCallBack_ = false;

  // initializing the video file writer, for saving results
  string strCurrTime = cfgParam_.GenLocalTimeStringNormal();
  if (cfgParam_.bUseSaveImgVidOnly)
    vidWriter_.open(cfgParam_.strVidSaveDstFileInfo + strCurrTime + ".mp4", VideoWriter::fourcc('m', 'p', '4', 'v'), 30,
                    cfgParam_.sizeFhdRaw);                 
}

CounterUasYoloV8P2InferLib::~CounterUasYoloV8P2InferLib()
{
  // Releasing the stream and buffers for tensorrt
  // yoloInfEng_.~YoloV8P2InferEngine();
}

// Callback function, raw image data
void CounterUasYoloV8P2InferLib::CbRawImgData(const sensor_msgs::msg::Image::ConstSharedPtr& msgRaw)
{
  //Grabbin the image frame, color image
  try
  {
    cvPtrImgSrc_ = cv_bridge::toCvCopy(msgRaw, sensor_msgs::image_encodings::BGR8);
    resize(cvPtrImgSrc_->image, imgRaw_, cfgParam_.sizeFhdRaw);
    bStartCamCallBack_ = true;
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "CbRawImgData::cv_bridge, color, exception: %s", e.what());
    bStartCamCallBack_ = false;
    return;
  }
}

// main loop: use ros compressed image topic
void CounterUasYoloV8P2InferLib::GenRosImgTopicLoop(Mat imgSrc)
{
  // making the source publish image and result image
  Mat imgRes;
  Mat imgRawPub;
  imgSrc.copyTo(imgRes);
  imgSrc.copyTo(imgRawPub);

  // setting 1-batch image
  vector<Mat> vecImgBatch;
  vecImgBatch.push_back(imgSrc);

  // calculating inference result
  yoloInfEng_.CalcInferenceResult(vecImgBatch, cfgParam_.fConfThresh, cfgParam_.fNmsThresh);

  // converting the recognition result frome vector<vector<Detection>> to vector<BboxInfo (assumption: single image)
  vector<BboxInfo> vecCurrRecogBboxInfo =
      yoloInfEng_.CvtRecogResDetection2BboxInfo(imgRes, yoloInfEng_.vecRecogDetectRes);

  // generating bboxes w.r.t the inference result
  miscCalc_.DrawVectorLabeledRect(imgRes, vecCurrRecogBboxInfo);

  // saving the result image slow rate with option
  SaveInfResImgLabel(imgRes, imgSrc, vecCurrRecogBboxInfo, cfgParam_.strRosImgFrameDstFolderInfo,
                     cfgParam_.strRosImgLabelDstFolderInfo, cfgParam_.strRosImgFrameSrcFolderInfo);

  // publishing the inferred bboxes information
  PubInferBboxes(vecCurrRecogBboxInfo, imgSrc, imgRes);

  // publishing the result image only, in debugging mode
  if (cfgParam_.bUseDebugMode)
  {
    auto msgImgPub = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imgRes).toImageMsg();
    pubDetResImg_->publish(*msgImgPub);
  }

  // setting waitKey
  waitKey(5);
  return;
}

// main loop: use video file
void CounterUasYoloV8P2InferLib::GenVidLoop(string strVidFile)
{
  // capturing the video file
  VideoCapture cap(strVidFile);

  // Check if camera opened successfully
  if (!cap.isOpened())
  {
    RCLCPP_ERROR(node_->get_logger(), "CounterUasYoloV8P2InferLib::Inference::Error opening video stream or file..");
    miscCalc_.bVidStatus = false;
    return;
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "CounterUasYoloV8P2InferLib::Inference::using video..");
    miscCalc_.bVidStatus = true;
  }

  // image processing loop using video file
  while (ok())
  {
    // capturing frame-by-frame
    Mat imgRaw;
    cap >> imgRaw;

    // checking framte status, empty->break immediately
    if (imgRaw.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "CounterUasYoloV8P2InferLib::Inference::Video is ended..");

      // releasing video writer
      if (cfgParam_.bUseSaveImgVidOnly)
        vidWriter_.release();
      break;
    }
    else
    {
      if (imgRaw.size().width != cfgParam_.sizeFhdRaw.width)
        resize(imgRaw, imgRaw, cfgParam_.sizeFhdRaw);
    }

    // making the source publish image and result image
    Mat imgRes;
    Mat imgRawPub;
    imgRaw.copyTo(imgRes);
    imgRaw.copyTo(imgRawPub);

    // setting 1-batch image
    vector<Mat> vecImgBatch;
    vecImgBatch.push_back(imgRaw);

    // calculating inference result
    yoloInfEng_.CalcInferenceResult(vecImgBatch, cfgParam_.fConfThresh, cfgParam_.fNmsThresh);

    // converting the recognition result frome vector<vector<Detection>> to vector<BboxInfo (assumption: single image)
    vector<BboxInfo> vecCurrRecogBboxInfo =
        yoloInfEng_.CvtRecogResDetection2BboxInfo(imgRes, yoloInfEng_.vecRecogDetectRes);

    // generating bboxes w.r.t the inference result
    miscCalc_.DrawVectorLabeledRect(imgRes, vecCurrRecogBboxInfo);

    // saving the result image slow rate with option
    SaveInfResImgLabel(imgRes, imgRaw, vecCurrRecogBboxInfo, cfgParam_.strVidImgDstFolderInfo,
                       cfgParam_.strVidLabelDstFolderInfo, cfgParam_.strVidImgSrcFolderInfo);

    // publishing the inferred bboxes information
    PubInferBboxes(vecCurrRecogBboxInfo, imgRaw, imgRes);

    // publishing the result image only, in debugging mode
    if (cfgParam_.bUseDebugMode)
    {
      auto msgImgPub = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imgRes).toImageMsg();
      pubDetResImg_->publish(*msgImgPub);

      // publishing the source image
      auto msgImgSrcPub = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imgRaw).toImageMsg();
      pubVidSrcImg_->publish(*msgImgSrcPub);
    }

    // setting waitKey
    waitKey(5);
  }
  return;
}

// main loop: use image folder
void CounterUasYoloV8P2InferLib::GenImgLoop(string strImgFolder)
{
  // 1st, resizing raw image and saving resized images
  // assigning variables for browsing raw images recursively
  vector<String> vecImgFileNm;
  glob(strImgFolder, vecImgFileNm, true);

  // browsing raw images recursively
  RCLCPP_INFO(node_->get_logger(), "CounterUasYoloV8P2InferLib::Inference::using images..");
  for (size_t i = 0; i < vecImgFileNm.size(); i++)
  {
    if (!rclcpp::ok()) 
    {
      RCLCPP_WARN(node_->get_logger(), "Shutdown requested, breaking GenImgLoop.");
      break;
    }
    // assigning the raw image and getting image width and height info
    Mat imgRaw = imread(vecImgFileNm[i]);
    if (imgRaw.size().width != cfgParam_.sizeFhdRaw.width)
      resize(imgRaw, imgRaw, cfgParam_.sizeFhdRaw);
    Size szImgRaw = miscCalc_.GenImgSize(imgRaw);
    Mat imgRes;
    imgRaw.copyTo(imgRes);

    // setting 1-batch image
    vector<Mat> vecImgBatch;
    vecImgBatch.push_back(imgRaw);

    // calculating inference result
    yoloInfEng_.CalcInferenceResult(vecImgBatch, cfgParam_.fConfThresh, cfgParam_.fNmsThresh);

    // converting the recognition result frome vector<vector<Detection>> to vector<BboxInfo (assumption: single image)
    vector<BboxInfo> vecCurrRecogBboxInfo =
        yoloInfEng_.CvtRecogResDetection2BboxInfo(imgRes, yoloInfEng_.vecRecogDetectRes);

    // generating bboxes w.r.t the inference result
    miscCalc_.DrawVectorLabeledRect(imgRes, vecCurrRecogBboxInfo);

    // saving inference result image
    if (cfgParam_.bUseSaveInfo)
    {
      // saving result with label file
      miscCalc_.SaveImgRes(imgRes, cfgParam_.strImgImgDstFolderInfo, cfgParam_.strImgLabelDstFolderInfo,
                           vecCurrRecogBboxInfo, true);
    }

    // for debugging
    imshow("imgRes", imgRes);
    waitKey(0);
  }
  return;
}

// saving inference result image and label
void CounterUasYoloV8P2InferLib::SaveInfResImgLabel(Mat imgRes, Mat imgSrc, vector<BboxInfo> vecInfo, string strImgDstInfo,
                                                string strLabelDstInfo, string strImgOnlyInfo)
{
  // saving the result image slow rate with option
  dTimeCounter_ += yoloInfEng_.dDt;
  if ((cfgParam_.bUseSaveInfo) && (dTimeCounter_ > cfgParam_.dSaveResImgDt))
  {
    miscCalc_.SaveImgRes(imgRes, strImgDstInfo, strLabelDstInfo, vecInfo, true);
    dTimeCounter_ = 0.0;

    if (cfgParam_.bUseSaveImgVidOnly)
      miscCalc_.SaveImgRaw(imgSrc, strImgOnlyInfo);
  }
  else
  {
    // for debugging
    // if (cfgParam_.bUseDebugMode)
    //   imshow("imgRes", imgRes);
  }

  // saving the result video file
  if (cfgParam_.bUseSaveImgVidOnly)
    vidWriter_.write(imgRes);

  // resetting the counter (forced)
  if (dTimeCounter_ > ((100.0) * (cfgParam_.dSaveResImgDt)))
    dTimeCounter_ = 0.0;
  return;
}

// publishing inferred bboxes information
void CounterUasYoloV8P2InferLib::PubInferBboxes(vector<BboxInfo> vecInput, Mat imgRaw, Mat imgDst)
{
  counter_uas_imgproc::msg::BoundingBoxes msgBboxes;
  msgBboxes.header.stamp = rclcpp::Clock().now();  // 현재 시간 설정
  msgBboxes.header.frame_id = "map";  // 필요에 따라 frame_id 설정

  for (auto i = 0; i < vecInput.size(); i++)
  {
    counter_uas_imgproc::msg::BoundingBox msgBbox;
    msgBbox.probability = (double)(vecInput[i].fConfidence);
    msgBbox.xmin = vecInput[i].rectBbox.tl().x;
    msgBbox.ymin = vecInput[i].rectBbox.tl().y;
    msgBbox.xmax = vecInput[i].rectBbox.br().x;
    msgBbox.ymax = vecInput[i].rectBbox.br().y;
    msgBbox.class_id = vecInput[i].labelInfo.nOrder;
    msgBbox.track_id = 99;
    msgBbox.class_info = vecInput[i].labelInfo.strLabel;
    msgBboxes.bounding_boxes.push_back(msgBbox);
  }

  auto msgImgRosSrc = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imgRaw).toImageMsg();
  auto msgImgRosRes = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imgDst).toImageMsg();
  
  msgImgRosSrc->header.stamp = msgBboxes.header.stamp;  // 동일한 timestamp 사용
  msgImgRosRes->header.stamp = msgBboxes.header.stamp;

  msgBboxes.img_ros_src = *msgImgRosSrc;
  msgBboxes.img_ros_res = *msgImgRosRes;

  pubDetResBboxes_->publish(msgBboxes);
}

// main loop function
void CounterUasYoloV8P2InferLib::MainInferLoop(int nMode)
{
  switch (nMode)
  {
    case USE_ROS_TOPIC: {
      if (bStartCamCallBack_)
        GenRosImgTopicLoop(imgRaw_);
      else
        RCLCPP_ERROR(node_->get_logger(),"Camera callback not started.");
      break;
    }
    case USE_VIDEO_FILE: {
      GenVidLoop(cfgParam_.strVidSrcFileInfo);
      break;
    }
    case USE_IMAGE_FILES: {
      GenImgLoop(cfgParam_.strImgSrcFolderInfo);
      break;
    }
    default: {
      RCLCPP_ERROR(node_->get_logger(), "CounterUasYoloV8P2InferLib::please check your option..:UsageCase");
      break;
    }
  }
  return;
}
