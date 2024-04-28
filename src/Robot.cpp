#include "Robot.h"

void int_enc_Rmot(){
  if (digitalRead(kMotEncPin0B)) {
      count_R--;
    } else {
      count_R++;
    }
}

void int_enc_Lmot(){
  if (digitalRead(kMotEncPin1B)) {
      count_L--;
    } else {
      count_L++;
    }
}

bool updateEncodersState(struct repeating_timer *t){
  (void) t;
  static bool running = true;
  if(!running){
    count_L=0;
    count_R=0;
    attachInterrupt(digitalPinToInterrupt(kMotEncPin1A), int_enc_Lmot, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kMotEncPin0A), int_enc_Rmot, CHANGE);
    running = true;
  }else{
    vel_R=(count_R*pulse2rad);//calculations to get rad/s from pulse count
    vel_L=(count_L*pulse2rad);
    detachInterrupt(digitalPinToInterrupt(kMotEncPin1A));
    detachInterrupt(digitalPinToInterrupt(kMotEncPin0A));
    running = false;
  }
  return true;
}


void Robot::init() {
  uint8_t i;

  // Encoders
  initEnc();
  //updateEncodersState();
  for (i = 0; i < 2; i++) {
    vel[i] = 0;
  }
  ITimer0.attachInterruptInterval(20 * 1000, updateEncodersState);// update wheels speed every 20ms
  // Motors
  
  mot[0].init(Motor1_IN1, Motor1_IN2, E_Motor1);
  mot[1].init(Motor2_IN3, Motor2_IN4, E_Motor2);

  // Controllers
  for (i = 0; i < 2; i++) {
    initCtrlPID(i);
  }
}

void Robot::update(uint32_t &delta) {
  uint8_t i;
  dt = delta;
  vel[0]=vel_R;
  vel[1]=vel_L;
  
  // Controllers
  pid[0].update(vel[0]);
  pid[1].update(vel[1]);

  // Actuators
  for (i = 0; i < 2; i++) {
    mot[i].setPWM( round( kMotV2MotPWM * pid[i].m ) );
  }
}

void Robot::stop(void) {
  uint8_t i;

  for (i = 0; i < 2; i++) {
    setMotorPWM(i, 0);
  }
}

void Robot::odometry(void)
{
  float v1e = mot[0].vel * wheel_radius_right;                 //change to separate wheel radiuses for left and right
  float v2e = mot[1].vel * wheel_radius_left;     

  // Estimate robot speed
  ve = (v1e + v2e) / 2.0;
  we = (v1e - v2e) / wheel_dist;
  
  // Estimate the distance and the turn angle
  float ds = ve * dt;
  float dtheta = we * dt;

  // Estimate pose
  xe += ds * cos(thetae + dtheta/2);
  ye += ds * sin(thetae + dtheta/2);
  thetae = thetae + dtheta;

  // Relative displacement
  rel_s += ds;
  rel_theta += dtheta;
}

void Robot::setRobotVW(float v, float w)
{
  float v1ref = v + w * wheel_dist / 2;
  float v2ref = v - w * wheel_dist / 2; 
  
  pid[0].w_ref = v1ref * wheel_radius_right;
  pid[1].w_ref = v2ref * wheel_radius_left;
}

void Robot::setMotorWref(uint8_t index, float new_w_r) {
  pid[index].w_ref = new_w_r;
}

void Robot::setMotorPWM(uint8_t index, int16_t pwm) {
  mot[index].setPWM(pwm);
}

void Robot::initEnc() {
  pinMode(kMotEncPin0A, INPUT_PULLUP);
  pinMode(kMotEncPin0B, INPUT_PULLUP);
  pinMode(kMotEncPin1A, INPUT_PULLUP);
  pinMode(kMotEncPin1B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(kMotEncPin0A), int_enc_Rmot, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kMotEncPin1A), int_enc_Lmot, CHANGE);
}

void Robot::initCtrlPID(uint8_t index) {

  pid[index].kp = kMotCtrlKc;
  if (kMotCtrlTi == 0) {
    pid[index].ki = 0;
  } else {
    pid[index].ki = kMotCtrlKc / kMotCtrlTi;
  }
  pid[index].kd = 0;
  pid[index].kf = kMotCtrlKf;
  pid[index].dt = kMotCtrlTime;

  pid[index].m_max = kMotVmax;

  pid[index].hamm_vd = 0;
  pid[index].hamm_v0 = 0;

  pid[index].reset();
}
