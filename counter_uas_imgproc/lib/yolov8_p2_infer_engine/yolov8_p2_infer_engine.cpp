#include "yolov8_p2_infer_engine/yolov8_p2_infer_engine.h"

using namespace std;
using namespace rclcpp;
using namespace cv;
using namespace nvinfer1;

YoloV8P2InferEngine::YoloV8P2InferEngine(const ConfigParamDetection& cfg, std::shared_ptr<rclcpp::Node> node)
 : cfgParam_(cfg), node_(node), miscCalc_(cfg, node), miscFunc_(cfg, node)
{
  // setting the gpu id
  cudaSetDevice(kGpuId);

  // calculating output size
  KOutputSize_ = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;

  // initializing variables
  runtime = nullptr;
  engine = nullptr;
  context = nullptr;
  nModelBboxes = 0;
  fOutBufferHost = nullptr;
  fDecodePtrHost = nullptr;
  fDecodePtrDevice = nullptr;
  vecRecogDetectRes.clear();
}

// YoloV8P2InferEngine::~YoloV8P2InferEngine()
// {
//   // releasing stream and buffers
//   cudaStreamDestroy(stream);
//   CUDA_CHECK(cudaFree(fDeviceBuffers[0]));
//   CUDA_CHECK(cudaFree(fDeviceBuffers[1]));
//   CUDA_CHECK(cudaFree(fDecodePtrDevice));
//   delete[] fDecodePtrHost;
//   delete[] fOutBufferHost;
//   cuda_preprocess_destroy();

//   // destroying the engine
//   delete context;
//   delete engine;
//   delete runtime;
// }

YoloV8P2InferEngine::~YoloV8P2InferEngine()
{
    //RCLCPP_INFO(node_->get_logger(), "YoloV8P2InferEngine destructor: Releasing resources...");

    if (stream) {
        cudaStreamDestroy(stream);
        stream = nullptr;
    }

    if (fDeviceBuffers[0]) {
        CUDA_CHECK(cudaFree(fDeviceBuffers[0]));
        fDeviceBuffers[0] = nullptr;
    }
    if (fDeviceBuffers[1]) {
        CUDA_CHECK(cudaFree(fDeviceBuffers[1]));
        fDeviceBuffers[1] = nullptr;
    }
    if (fDecodePtrDevice) {
        CUDA_CHECK(cudaFree(fDecodePtrDevice));
        fDecodePtrDevice = nullptr;
    }
    if (fDecodePtrHost) {
        delete[] fDecodePtrHost;
        fDecodePtrHost = nullptr;
    }
    if (fOutBufferHost) {
        delete[] fOutBufferHost;
        fOutBufferHost = nullptr;
    }

    cuda_preprocess_destroy();

    if (context) {
        context->destroy();
        context = nullptr;
    }
    if (engine) {
        engine->destroy();
        engine = nullptr;
    }
    if (runtime) {
        runtime->destroy();
        runtime = nullptr;
    }

    //RCLCPP_INFO(node_->get_logger(), "YoloV8P2InferEngine destructor: Resources successfully released.");
}

// calculating the tensorrt engine information
bool YoloV8P2InferEngine::CalcTRTEngineInfo(string strFileNameEng)
{
  bool bRes = DeserializeEngine(strFileNameEng, &runtime, &engine, &context);
  CUDA_CHECK(cudaStreamCreate(&stream));
  cuda_preprocess_init(kMaxInputImageSize);
  auto outDims = engine->getBindingDimensions(1);
  nModelBboxes = outDims.d[0];
  return bRes;
}

// calculating bboxes result
void YoloV8P2InferEngine::CalcInferenceResult(vector<Mat> vecImgBatch, float fConfThresh, float fNmsThresh)
{
  // setting pre-processing, using batch image
  cuda_batch_preprocess(vecImgBatch, fDeviceBuffers[0], kInputW, kInputH, stream);

  // inferencing model
  Infer(*context, stream, (void**)fDeviceBuffers, fOutBufferHost, kBatchSize, fDecodePtrHost, fDecodePtrDevice,
        nModelBboxes, fConfThresh, fNmsThresh);

  // decoding and non-maximum supressing results using gpu device
  vecRecogDetectRes.clear();
  batch_process(vecRecogDetectRes, fDecodePtrHost, vecImgBatch.size(), bbox_element, vecImgBatch);
}

// inferencing the yolov8-p2 model, for recognizing illegal drone, Kari Antidrone System
void YoloV8P2InferEngine::Infer(IExecutionContext& context, cudaStream_t& stream, void** buffers, float* output,
                                int batchsize, float* fDecodePtrHost, float* fDecodePtrDevice, int nModelBboxes,
                                float fConfThresh, float fNmsThresh)
{
  // inferencing on the batch asynchronously, and DMA output back to host
  // inferencing and calculating non-maximum suppression
  auto start = std::chrono::system_clock::now();
  context.enqueue(batchsize, buffers, stream, nullptr);
  CUDA_CHECK(cudaMemsetAsync(fDecodePtrDevice, 0, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), stream));
  cuda_decode((float*)(buffers[1]), nModelBboxes, fConfThresh, fDecodePtrDevice, kMaxNumOutputBbox, stream);
  cuda_nms(fDecodePtrDevice, fNmsThresh, kMaxNumOutputBbox, stream);
  CUDA_CHECK(cudaMemcpyAsync(fDecodePtrHost, fDecodePtrDevice, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element),
                             cudaMemcpyDeviceToHost, stream));
  auto end = std::chrono::system_clock::now();

  // for debugging
  auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  dDt = (dt) / (1000.0);
  dHz = (1.0) / (dDt);
  if (cfgParam_.bUseDebugMode)
    RCLCPP_INFO(node_->get_logger(), "YoloV8P2InferEngine::Infer::inference and gpu postprocess::(time,Hz):(%ld[ms],%.4lf[Hz])",
             std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(), dHz);
  CUDA_CHECK(cudaStreamSynchronize(stream));
}

// deserializing the tensorrt engine file
bool YoloV8P2InferEngine::DeserializeEngine(string& strFileNameEng, IRuntime** runtime, ICudaEngine** engine,
                                            IExecutionContext** context)
{
  // checking the file status, early return
  ifstream file(strFileNameEng, std::ios::binary);
  if (!file.good())
  {
    RCLCPP_ERROR(node_->get_logger(), "YoloV8P2InferEngine::DeserializeEngine: read %s error!", strFileNameEng.c_str());
    return false;
  }

  // reading file status
  size_t size = 0;
  file.seekg(0, file.end);
  size = file.tellg();
  file.seekg(0, file.beg);
  char* chSerializedEngine = new char[size];
  assert(chSerializedEngine);
  file.read(chSerializedEngine, size);
  file.close();

  // deserializing the engine
  *runtime = createInferRuntime(gLogger_);
  assert(*runtime);
  *engine = (*runtime)->deserializeCudaEngine(chSerializedEngine, size);
  assert(*engine);
  *context = (*engine)->createExecutionContext();
  assert(*context);
  delete[] chSerializedEngine;

  return true;
}

// preparing buffer for inference
void YoloV8P2InferEngine::PrepareBuffer(ICudaEngine* engine)
{
  CalcBuffer(engine, &fDeviceBuffers[0], &fDeviceBuffers[1], &fOutBufferHost, &fDecodePtrHost, &fDecodePtrDevice);
  return;
}

// calculating buffer for inference
void YoloV8P2InferEngine::CalcBuffer(ICudaEngine* engine, float** fInBufferDevice, float** fOutBufferDevice,
                                     float** fOutBufferHost, float** fDecodePtrHost, float** fDecodePtrDevice)
{
  // checking the number of bindings, need to exactly 2
  // for debugging
  // ROS_INFO("engine->getNbBindings():%d", (int)(engine->getNbBindings()));
  assert(engine->getNbBindings() == 2);

  // in order to bind the buffers, need to know the names of the input and output tensors
  // need to guaranteed to be less than IEngine::getNbBindings()
  const int inputIndex = engine->getBindingIndex(kInputTensorName);
  const int outputIndex = engine->getBindingIndex(kOutputTensorName);
  assert(inputIndex == 0);
  assert(outputIndex == 1);

  // creating GPU buffers on device
  CUDA_CHECK(cudaMalloc((void**)(fInBufferDevice), kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
  CUDA_CHECK(cudaMalloc((void**)(fOutBufferDevice), kBatchSize * KOutputSize_ * sizeof(float)));

  // Allocate memory for decode_ptr_host and copy to device
  *fDecodePtrHost = new float[1 + kMaxNumOutputBbox * bbox_element];
  CUDA_CHECK(cudaMalloc((void**)fDecodePtrDevice, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element)));
}

// converting the recognition result frome vector<vector<Detection>> to vector<BboxInfo> (assumption: single image)
vector<BboxInfo> YoloV8P2InferEngine::CvtRecogResDetection2BboxInfo(Mat imgRes, vector<vector<Detection>> vecDetection)
{
  vector<BboxInfo> vecRes;
  if (vecDetection[0].size() > 0)
  {
    for (size_t i = 0; i < vecDetection[0].size(); i++)
    {
      BboxInfo bboxInfo;
      bboxInfo.nLabel = vecDetection[0][i].class_id;
      if (bboxInfo.nLabel == 0)  // drone
        bboxInfo.labelInfo = cfgParam_.vecAntiDroneLabelInfo[0];
      else if (bboxInfo.nLabel == 1)  // bird
        bboxInfo.labelInfo = cfgParam_.vecAntiDroneLabelInfo[1];
      else  // background, etc.
        bboxInfo.labelInfo = cfgParam_.vecAntiDroneLabelInfo[9];
      bboxInfo.fConfidence = vecDetection[0][i].conf;
      bboxInfo.rectBbox = get_rect(imgRes, vecDetection[0][i].bbox);
      bboxInfo.nWidth = bboxInfo.rectBbox.width;
      bboxInfo.nHeight = bboxInfo.rectBbox.height;
      bboxInfo.ptLt = bboxInfo.rectBbox.tl();
      bboxInfo.ptRb = bboxInfo.rectBbox.br();
      bboxInfo.ptCen = Point((int)(vecDetection[0][i].bbox[0]), (int)(vecDetection[0][i].bbox[1]));
      bboxInfo.fYoloData[0] = (vecDetection[0][i].bbox[0]) / (cfgParam_.sizeFhdRaw.width);
      bboxInfo.fYoloData[1] = (vecDetection[0][i].bbox[1]) / (cfgParam_.sizeFhdRaw.height);
      bboxInfo.fYoloData[2] = (vecDetection[0][i].bbox[2]) / (cfgParam_.sizeFhdRaw.width);
      bboxInfo.fYoloData[3] = (vecDetection[0][i].bbox[3]) / (cfgParam_.sizeFhdRaw.height);
      vecRes.push_back(bboxInfo);
    }
  }
  else
  {
    BboxInfo bboxInfo;
    bboxInfo.nLabel = -1;      // no detection
    if (bboxInfo.nLabel == 0)  // drone
      bboxInfo.labelInfo = cfgParam_.vecAntiDroneLabelInfo[0];
    else if (bboxInfo.nLabel == 1)  // bird
      bboxInfo.labelInfo = cfgParam_.vecAntiDroneLabelInfo[1];
    else  // background, etc.
      bboxInfo.labelInfo = cfgParam_.vecAntiDroneLabelInfo[9];
    bboxInfo.fConfidence = 0.0;  // no detection
    bboxInfo.rectBbox = Rect(0, 0, 0, 0);
    bboxInfo.nWidth = 0;
    bboxInfo.nHeight = 0;
    bboxInfo.ptLt = Point(0, 0);
    bboxInfo.ptRb = Point(0, 0);
    bboxInfo.ptCen = Point(0, 0);
    bboxInfo.fYoloData[0] = 0.0;
    bboxInfo.fYoloData[1] = 0.0;
    bboxInfo.fYoloData[2] = 0.0;
    bboxInfo.fYoloData[3] = 0.0;
    vecRes.push_back(bboxInfo);
  }
  return vecRes;
}