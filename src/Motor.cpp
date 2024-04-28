#include "Motor.h"

void Motor::init(int in1, int in2, int enable) {

  IN1=in1;
  IN2=in2;
  EN = enable;
  /*PWM_fwr = new RP2040_PWM(IN1, 20000, 0);
  PWM_fwr = new RP2040_PWM(IN1, 20000, 0);*/

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN, OUTPUT);

  //enable = true;
  pwm = 0;
  vel =0;
  setPWM(0);
}

void Motor::vel_to_pwm() {
  if (vel == 0)
    pwm = 0;
  else
    pwm = constrain(abs(vel) * kvel2PWM,pwm_offset,255);

}

void Motor::setPWM(int16_t new_vel) {
  vel=new_vel;
  vel_to_pwm();
  if (vel > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  analogWrite(EN, pwm);
}
