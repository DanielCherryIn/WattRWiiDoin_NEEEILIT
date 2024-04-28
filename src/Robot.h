#ifndef ROBOT_H
#define ROBOT_H

#include <RPi_Pico_TimerInterrupt.h>
//#include <TimerOne.h>

#include "robotconfig.h"
#include "Motor.h"
#include "CtrlPID.h"
#include <math.h>

extern volatile int count_R;// count for right motor encoder
extern volatile int count_L;// count for left motor encoder
extern volatile float vel_R;
extern volatile float vel_L;

class Robot {
 public:
  uint32_t dt;
  volatile float vel[2];//0->right//1->left
  Motor mot[2];
  CtrlPID pid[2];

  RPI_PICO_Timer ITimer0 = RPI_PICO_Timer(0);

 public:
  void init();

  void update(uint32_t &delta);
  //void send(void);
  void stop(void);

  void setMotorWref(uint8_t index, float new_w_r);
  void setMotorPWM(uint8_t index, int16_t pwm);

 private:
  void initEnc();
  void initCtrlPID(uint8_t index);
};

#endif
