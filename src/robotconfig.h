#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#include <Arduino.h>


/******************************************************************************
 * Robot parameters
 * - kinematic configuration
 * - battery
 ******************************************************************************/
const float wheel_dist = 0.105; //wheel distance (10.5cm)
const float wheel_radius_right = 0.065/2; // 3.25cm
const float wheel_radius_left = 0.065/2; // 3.25cm
const float kRobotBattVnom = 7.4;    //!< nominal battery level (V)

/******************************************************************************
 * General inputs / outputs
 * - ultra sonic sensors
 * - L293DNE
 ******************************************************************************/
const int Motor1_IN1 = 27;
const int Motor1_IN2 = 22;
const int Motor2_IN3 = 26;
const int Motor2_IN4 = 28;
const int E_Motor1 = 15;
const int E_Motor2 = 14;
const int Ultrasonic_TRIG1 = 0;
const int Ultrasonic_TRIG2 = 1;
const int Ultrasonic_TRIG3 = 2;
const int Ultrasonic_ECHO1 = 3;
const int Ultrasonic_ECHO2 = 4;
const int Ultrasonic_ECHO3 = 5;

/******************************************************************************
 * Motor parameters
 * - gear reduction ratio and encoders resolution
 * - controllers parameters
 ******************************************************************************/
const float kMotNgear  = 48.0;    //!< gear reduction ratio (n:1)
const float kMotEncRes = 16*4;    //!< encoder resolution (tick count per rev.)

const uint8_t kMotEncPin0A = 10;  //!< encoder channel A of right wheel
const uint8_t kMotEncPin0B = 11;  //!< encoder channel B of right wheel
const uint8_t kMotEncPin1A = 12;  //!< encoder channel A of left wheel
const uint8_t kMotEncPin1B = 13;  //!< encoder channel B of left wheel


const float kMotModelKp  = 4.5000;    //!< gain (rad.s^(-1) / V)
const float kMotModelTau = 0.1000;    //!< time constant (s)
const float kMotModelLag = 0.0000;    //!< lag lag (s)

// PWM constants

const unsigned long kMotCtrlFreq = 50UL;      //!< frequency (Hz)
const float kMotCtrlTime = 1.0 / kMotCtrlFreq;//!< period (s)
const unsigned long kMotCtrlTimeUs = 1000000UL / kMotCtrlFreq;
const unsigned long kMotCtrlTimeout = 100UL;  //!< watchdog timeout (ms)
const bool kMotCtrlTimeoutEnable = true;      //!< enable watchdog (true/false)

const unsigned long kMotCtrlLEDOkFreq = 4UL;  //!< heartbeat LED frequency (Hz)
const unsigned long kMotCtrlLEDOkCount =
    1000000UL / kMotCtrlLEDOkFreq / kMotCtrlTimeUs / 2;


//
const float kMotVmax = 7.4;          //!< maximum voltage appliable to motors (V)
const int16_t kMotPWMmax = 255;     //!< maximum PWM (0..1023)
const int16_t kMotPWMDeltaMax = 100;//!< maximum variation in PWM (0..1023)
const float pwm_offset = 50;
const float pulse2rad = 2*PI/19099.0249/0.02; // pulse count to rad/s constant
const bool kMotPWMDeltaMaxEnabled = true; //!< enable limits on PWM variation

const float kMotHammV0 = 0.20;         //!< estimated motors' deadzone (V)
const float kMotHammVd = 0.12;         //!< compensated motors' deadzone (V)

//! IMC tunning: desired time constant for the closed-loop (s)
const float kMotCtrlTauCl = kMotModelTau / 1.0;
//! IMC tunning: Kc_PI * Kp_plant
const float kMotCtrlKcKp = kMotModelTau / (kMotCtrlTauCl + kMotModelLag);
//! PI proportional gain (V / rad.s^(-1))
const float kMotCtrlKc = kMotCtrlKcKp / kMotModelKp;
//! PI integration time (s)
const float kMotCtrlTi = kMotModelTau;
//! Feed-Forward constant
const float kMotCtrlKf = 0.456294584;

/******************************************************************************
 * Conversion constants
 ******************************************************************************/
//! Conversion constant: encoder pulses (ticks) > motor angular speed (rad/s)
const float kEncImp2MotW =
    2 * PI * 1000000 / (1.0 * kMotCtrlTimeUs * kMotNgear * kMotEncRes);
//! Conversion constant: vel -> PWM (0..PWM_max)
const float kvel2PWM = kMotPWMmax / 100; 
//! Conversion constant: motor voltage (V) > PWM (0..PWM_max)
const float kMotV2MotPWM = kMotPWMmax * 1.0 / kRobotBattVnom;

#endif
