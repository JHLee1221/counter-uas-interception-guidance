#include "yaw_rate_ctrl.h"

using namespace std;

CounterUASGuidanceYawRateCtrl::CounterUASGuidanceYawRateCtrl(int nFrameRate, double dKp, double dKi, double dKd, double dAdpGain, double dAlpha, double dYawAngLmt, double dYawRateLmt)
  : dt_(1.0 / static_cast<double>(nFrameRate))
  , dYawErr_(0.0)
  , dYawErrI_(0.0)
  , dYawErrD_(0.0)
  , dPrevYawErr_(0.0)
  , dRawYawRate_(0.0)
  , dFltYawRate_(0.0)
  , dAdptiveKp_(0.0)
  , dKp_(dKp)
  , dKi_(dKi)
  , dKd_(dKd)
  , dAdpGain_(dAdpGain)
  , dAlpha_(dAlpha)
  , dYawAngLmt_(dYawAngLmt)
  , dYawRateLmt_(dYawRateLmt)
{
}

CounterUASGuidanceYawRateCtrl::~CounterUASGuidanceYawRateCtrl()
{
}

double CounterUASGuidanceYawRateCtrl::dUpdateYawRate(double dUasYawAng, double dLosYawAng)
{
  dYawErr_ = dWrapToPi(dLosYawAng - dUasYawAng);

  dYawErrI_ += dYawErr_ * dt_;
  dYawErrD_ = (dYawErr_ - dPrevYawErr_) / dt_;
  dPrevYawErr_ = dYawErr_;

  dAdptiveKp_ = dKp_ + dAdpGain_ * tanh(abs(dYawErr_));

  if (abs(dYawErr_) < dYawAngLmt_ * D2R)
  {
    return dYawErr_ = 0.0;
  }
  else
  {
    dRawYawRate_ = dAdptiveKp_ * dYawErr_ + dKi_ * dYawErrI_ + dKd_ * dYawErrD_;
  }

  dFltYawRate_ = (1.0 - dAlpha_) * dRawYawRate_ + dAlpha_ * dFltYawRate_;
  return std::clamp(dFltYawRate_, -dYawRateLmt_, dYawRateLmt_);
}

double CounterUASGuidanceYawRateCtrl::dGetLastYawErr() const
{
  return dPrevYawErr_;
}

double CounterUASGuidanceYawRateCtrl::dWrapToPi(double dAngle)
{
  return fmod(dAngle + PI, 2.0 * PI) - PI;
}

