#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "global_header.h"

class pidController
{
public:
  pidController(float pGain, float iGain, float dGain, float dt);
  void updateConstants(float pGain, float iGain, float dGain);
  void reset();

  void calculate(float x_error, float y_error, float& x_output, float& y_output);

private:
  float Kp_, Ki_, Kd_;                  // PID gains
  float x_prev_error_, y_prev_error_;   // previous errors
  const float x_proportional_, y_proportional_; // proportional term
  float x_integral_, y_integral_;       // integral term
  const float x_derivative_, y_derivative_;   // derivative term
  const float dt_;                      // time step

};
#endif // PID_CONTROLLER_H