#ifndef COUNTER_UAS_GUIDANCE_YAW_RATE_CTRL_H
#define COUNTER_UAS_GUIDANCE_YAW_RATE_CTRL_H

#include "global_header.h"

class CounterUASGuidanceYawRateCtrl
{
public:
  CounterUASGuidanceYawRateCtrl(int nFrameRate, double dKp, double dKi, double dKd, double dAdpGain, double dAlpha, double dYawAngLmt, double dYawRateLmt);
  ~CounterUASGuidanceYawRateCtrl();

  double dUpdateYawRate(double dUasYawAng, double dLosYawAng);
  double dGetLastYawErr() const;
  static double dWrapToPi(double dAngle);

private:  
  double dt_;
  double dYawErr_;
  double dYawErrI_;
  double dYawErrD_;
  double dPrevYawErr_;
  double dRawYawRate_;
  double dFltYawRate_;
  double dAdptiveKp_;
  double dKp_;
  double dKi_;
  double dKd_;
  double dAdpGain_;
  double dAlpha_;
  double dYawAngLmt_;
  double dYawRateLmt_;
};

#endif // COUNTER_UAS_GUIDANCE_YAW_RATE_CTRL_H