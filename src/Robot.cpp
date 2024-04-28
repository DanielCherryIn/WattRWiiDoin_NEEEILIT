#include "Robot.h"

void int_enc_Rmot(){
  if (digitalRead(kMotEncPin0B)) {
      count_R++;
    } else {
      count_R--;
    }
}

void int_enc_Lmot(){
  if (digitalRead(kMotEncPin1B)) {
      count_L++;
    } else {
      count_L--;
    }
}

bool updateEncodersState(){
  unsigned long c=millis();
  
  while((millis()-c)<10){
    if(digitalRead(kMotEncPin0A) && (!digitalRead(kMotEncPin0B)))
      int_enc_Rmot();

    if(digitalRead(kMotEncPin1A) && (!digitalRead(kMotEncPin1B)))
      int_enc_Lmot();  
  }
  vel_R=(count_R*pulse2rad);//calculations to get rad/s from pulse count
  vel_L=(count_L*pulse2rad);
  count_L=0;
  count_R=0;
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
  //ITimer0.attachInterruptInterval(20 * 1000, updateEncodersState);// update wheels speed every 20ms
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

  updateEncodersState();

  /* vel[0]=abs(vel_R)*kMotV2MotPWM * pid[0].m/abs(kMotV2MotPWM * pid[0].m);
  vel[1]=abs(vel_L)*kMotV2MotPWM * pid[1].m/abs(kMotV2MotPWM * pid[1].m);
  
  /* // Controllers
  pid[0].update(vel[0]);
  pid[1].update(vel[1]); 

  // Actuators
  for (i = 0; i < 2; i++) {
    mot[i].setPWM( round( kMotV2MotPWM * vel[0]) );
  } */
}

void Robot::stop(void) {
  uint8_t i;

  for (i = 0; i < 2; i++) {
    setMotorPWM(i, 0);
  }
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
