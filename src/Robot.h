#ifndef ROBOT_H
#define ROBOT_H

#include <RPi_Pico_TimerInterrupt.h>
//#include <TimerOne.h>

#include "robotconfig.h"
#include "Motor.h"
//#include "CtrlPID.h"
#include <math.h>

#include "Graph/graph.h"

extern volatile int count_R;// count for right motor encoder
extern volatile int count_L;// count for left motor encoder
extern volatile float vel_R;
extern volatile float vel_L;

class Robot {
 public:
  uint32_t dt;
  volatile float vel[2];//0->right//1->left
  Motor mot[2];
  //CtrlPID pid[2];

  direction_t dir;
  node* cur;

  bool wall_R, wall_F, wall_L;

  int sonic_dist[3]; //stores distance from each sensor (0->right, 1->left, 2->front)
  const int wall_dist = 5 ;   //distance to be from wall when only one side (cm)
  const int wall_thresh = 14; //distance to consider there is a wall on that side (if over thresh, will only follow one side) 
  float sum_error;
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

  void spin_right(int speed);
  void spin_left(int speed);

  void setMotorWref(uint8_t index, float new_w_r);
  void setMotorPWM(uint8_t index, int16_t pwm);

  void odometry(void);    //calculates robot linear and angular speed, positioning and movement
  void setRobotVW(float v, float w);  //sets motors Wref based on intended robot linear and angular speed 

  void followWall(float v, float k, float ki, float deltaT);  // goes in a straight line at speed v while aligning with wall(s)

 private:
  void initEnc();
  void initCtrlPID(uint8_t index);
};

#endif
