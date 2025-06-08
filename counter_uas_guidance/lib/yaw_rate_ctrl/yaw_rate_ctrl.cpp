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

  double newYawErr = dWrapToPi(dLosYawAng - dUasYawAng);

  if ((newYawErr * dPrevYawErr_ < 0.0) && std::abs(newYawErr) > 170.0 * D2R) {
    newYawErr = dPrevYawErr_;
  }
  dYawErr_ = newYawErr;

  // Anti-windup
  double int_limit = D2R * 20.0;
  dYawErrI_ += dYawErr_ * dt_;
  dYawErrI_ = std::clamp(dYawErrI_, -int_limit, int_limit);

  // Derivative
  dYawErrD_ = (dYawErr_ - dPrevYawErr_) / dt_;
  dPrevYawErr_ = dYawErr_;

  // Adaptive Kp with upper bound
  dAdptiveKp_ = std::clamp(dKp_ + dAdpGain_ * tanh(std::abs(dYawErr_)), 0.0, 2.0);

  // Deadband
  if ((std::abs(dYawErr_) < D2R * 2.0) && (abs(dFltYawRate_) < D2R) * 1.0) 
  {
    dYawErrI_ = 0.0;
    return dFltYawRate_ = 0.0;
  }

  // PID and filter
  dRawYawRate_ = dAdptiveKp_ * dYawErr_ + dKi_ * dYawErrI_ + dKd_ * dYawErrD_;
  dFltYawRate_ = (1.0 - dAlpha_) * dRawYawRate_ + dAlpha_ * dFltYawRate_;
  return std::clamp(dFltYawRate_, -dYawRateLmt_, dYawRateLmt_);

}

double CounterUASGuidanceYawRateCtrl::dGetLastYawErr() const
{
  return dPrevYawErr_;
}

double CounterUASGuidanceYawRateCtrl::dWrapToPi(double dAngle)
{
  while (dAngle > M_PI) dAngle -= 2.0 * M_PI;
  while (dAngle < -M_PI) dAngle += 2.0 * M_PI;

  if (std::abs(dAngle - M_PI) < 1e-6 || std::abs(dAngle + M_PI) < 1e-6)
  dAngle = 0.0;
  return dAngle;
}

