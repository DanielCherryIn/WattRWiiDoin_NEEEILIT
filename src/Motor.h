#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>


#include "robotconfig.h"

class Motor {
 public:
  //bool enable;
  
  float pwm, vel;

  //RP2040_PWM *PWM_fwr, *PWM_rev;

  int16_t IN1, IN2, EN; // IN1 -> foward pin / IN2 -> reverse pin // Enable pin

 public:
  void init(int in1, int in2, int enable);
  void setPWM(int16_t new_pwm);
  void vel_to_pwm();
};

#endif
