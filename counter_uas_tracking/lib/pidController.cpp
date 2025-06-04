#include "pidController.h"

pidController::pidController(float pGain, float iGain, float dGain, float dt)
  : Kp_(pGain)
  , Ki_(iGain)
  , Kd_(dGain)
  , dt_(dt)
  , x_prev_error_(0.0f)
  , y_prev_error_(0.0f)
  , x_integral_(0.0f)
  , y_integral_(0.0f)
{
  if (dt_ <= 0.0f) 
  {
    throw std::runtime_error("dt must be greater than 0");
  }
}

void pidController::updateConstants(float pGain, float iGain, float dGain)
{
  Kp_ = pGain;
  Ki_ = iGain;
  Kd_ = dGain;
}

void pidController::reset()
{
  x_prev_error_ = 0.0f;
  y_prev_error_ = 0.0f;
  x_integral_ = 0.0f;
  y_integral_ = 0.0f;
}

void pidController::calculate(float x_error, float y_error, float& x_output, float& y_output)
{
  // Proportional term
  x_proportional_ = Kp_ * x_error;
  y_proportional_ = Kp_ * y_error;

  // Integral term
  x_integral_ += Ki_ * x_error * dt_;
  y_integral_ += Ki_ * y_error * dt_;

  // Derivative term
  x_derivative_ = Kd_ * (x_error - x_prev_error_) / dt_;
  y_derivative_ = Kd_ * (y_error - y_prev_error_) / dt_;

  // Calculate output
  x_output = x_proportional + x_integral_ + x_derivative;
  y_output = y_proportional + y_integral_ + y_derivative;

  // Save previous error
  x_prev_error_ = x_error;
  y_prev_error_ = y_error;
}