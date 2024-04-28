#include <Arduino.h>
#include <Robot.h>


volatile int count_R = 0;// count for right motor encoder
volatile int count_L = 0;// count for left motor encoder
volatile float vel_R = 0;
volatile float vel_L = 0;

unsigned long now;

unsigned long LOOP_TIME = 40;

Robot robot;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  robot.init();
  pinMode(LED_BUILTIN, OUTPUT);
  now=millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  /*digitalWrite(LED_BUILTIN, HIGH);

  delay(1000);

  digitalWrite(LED_BUILTIN, LOW);

  delay(1000);*/
  if((millis()-now)>LOOP_TIME){
    now=millis();
    Serial.println(vel_L);
    robot.setMotorPWM(0,100);
    robot.setMotorPWM(1,100);
  }

}
