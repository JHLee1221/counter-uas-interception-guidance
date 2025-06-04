#include PID_CTRL_H
#define PID_CTRL_H

#include "global_header.h"

class pidCtrl
{
public: 
  PIDCtrl(float kp, float ki, float kd, float dt)

  void updateConstants(float kp, float ki, float kd);
  
}