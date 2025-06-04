#include "calVFov.h"

CameraParameters::CameraParameters(float fHfovDeg, int nImgWidth, int nImgHeight)
  : fHFovDeg_(fHfovDeg)
  , nImgWidth_(nImgWidth)
  , nImgHeight_(nImgHeight)
{
  if (fHFovDeg_ <= 0.0f || nImgWidth_ <= 0 || nImgHeight_ <= 0) 
  {
    throw std::invalid_argument("Invalid camera parameters");
  }
}

float CameraParameters::CalVFov()
{
  fHFovRad = DEG2RAD * fHFovDeg_;
  fAR = static_cast<float>(nImgWidth_) / static_cast<float>(nImgHeight_);
  fVFovRad = 2.0f * atan(tan(fHFovRad / 2.0f) / fAR);
  return fVFovRad;
}