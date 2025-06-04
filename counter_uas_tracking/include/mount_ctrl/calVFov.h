#ifndef CAL_VFOV_H
#define CAL_VFOV_H

#include "global_header.h" 

class CameraParameters
{
public:
  CameraParameters(float fHFovDeg, int nImgWidth, int nImgHeight);

  float CalVFov();

private:
  float fHFovDeg_;
  int nImgWidth_;
  int nImgHeight_;
  float fHFovRad;
  float fAR;
  float fVFovRad;
};

#endif // CAL_VFov_H