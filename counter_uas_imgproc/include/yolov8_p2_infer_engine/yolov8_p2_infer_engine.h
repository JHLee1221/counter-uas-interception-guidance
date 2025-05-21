#ifndef KARI_DRONECOP_RD_IMGPROC_YOLOV8_P2_INFER_ENGINE_H
#define KARI_DRONECOP_RD_IMGPROC_YOLOV8_P2_INFER_ENGINE_H

#include "global_header.h"
#include "config_param.h"
#include "misc_calc_lib.h"
#include "misc_func.h"

using namespace std;
using namespace rclcpp;
using namespace cv;
using namespace nvinfer1;

class CustomLogger : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char* msg) noexcept override {
    // 로깅 심각도가 kERROR보다 낮으면 무시
    if (severity <= Severity::kERROR) {
      std::cerr << "TensorRT Logger: " << msg << std::endl;
    }
  }
};

class YoloV8P2InferEngine
{
public:
  YoloV8P2InferEngine(const ConfigParam& cfg, std::shared_ptr<rclcpp::Node> node);
  ~YoloV8P2InferEngine();

  void PrepareBuffer(ICudaEngine* engine);
  void CalcInferenceResult(vector<Mat> vecImgBatch, float fConfThresh, float fNmsThresh);
  bool CalcTRTEngineInfo(string strFileNameEng);
  vector<BboxInfo> CvtRecogResDetection2BboxInfo(Mat imgRes, vector<vector<Detection>> vecDetection);

  IRuntime* runtime;
  ICudaEngine* engine;
  IExecutionContext* context;
  cudaStream_t stream;
  int nModelBboxes;

  float* fDeviceBuffers[2];
  float* fOutBufferHost = nullptr;
  float* fDecodePtrHost = nullptr;
  float* fDecodePtrDevice = nullptr;

  vector<vector<Detection>> vecRecogDetectRes;

  double dDt;
  double dHz;

private:
  ConfigParam cfgParam_;
  MiscCalc miscCalc_;
  MiscFunction miscFunc_;
  CustomLogger gLogger_;

  std::shared_ptr<rclcpp::Node> node_;

  int KOutputSize_;

  bool DeserializeEngine(string& strFileNameEng, IRuntime** runtime, ICudaEngine** engine, IExecutionContext** context);
  void CalcBuffer(ICudaEngine* engine, float** fInBufferDevice, float** fOutBufferDevice, float** fOutBufferHost,
                  float** fDecodePtrHost, float** fDecodePtrDevice);
  void Infer(IExecutionContext& context, cudaStream_t& stream, void** buffers, float* output, int batchsize,
             float* fDecodePtrHost, float* fDecodePtrDevice, int nModelBboxes, float fConfThresh, float fNmsThresh);
};

#endif  // KARI_DRONECOP_RD_IMGPROC_YOLOV8_P2_INFER_ENGINE_H
