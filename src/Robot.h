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

  float ve;   //estimated robot linear speed
  float we;   //estimated robot angular speed
  float xe, ye, thetae; //relative positioning (x, y, angle)
  float rel_s, rel_theta; //relative movement (linear, angular)

  RPI_PICO_Timer ITimer0 = RPI_PICO_Timer(0);

 public:
  void init();

  void update(uint32_t &delta);
  //void send(void);
  void stop(void);

  void setMotorWref(uint8_t index, float new_w_r);
  void setMotorPWM(uint8_t index, int16_t pwm);

  void odometry(void);    //calculates robot linear and angular speed, positioning and movement
  void setRobotVW(float v, float w);  //sets motors Wref based on intended robot linear and angular speed 

 private:
  void initEnc();
  void initCtrlPID(uint8_t index);
};

#endif
