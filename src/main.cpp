#include <Arduino.h>
#include <Robot.h>
#include <HC_SR04.h>

volatile int count_R = 0;// count for right motor encoder
volatile int count_L = 0;// count for left motor encoder
volatile float vel_R = 0;
volatile float vel_L = 0;

unsigned long now;

unsigned long LOOP_TIME = 40;

Robot robot;

//============ ULTRASONIC sensor =============//

#define ULTRASONIC_TRIG_1 0
#define ULTRASONIC_ECHO_1 1

#define ULTRASONIC_TRIG_2 2
#define ULTRASONIC_ECHO_2 3

#define ULTRASONIC_TRIG_3 4
#define ULTRASONIC_ECHO_3 5

HC_SR04 sensor1(ULTRASONIC_TRIG_1, ULTRASONIC_ECHO_1, digitalPinToInterrupt(ULTRASONIC_ECHO_1));
HC_SR04 sensor2(ULTRASONIC_TRIG_2, ULTRASONIC_ECHO_2, digitalPinToInterrupt(ULTRASONIC_ECHO_2));
HC_SR04 sensor3(ULTRASONIC_TRIG_3, ULTRASONIC_ECHO_3, digitalPinToInterrupt(ULTRASONIC_ECHO_3));
int dist1, dist2, dist3;
//===========================================//

//================= WIFI ====================//
#include <ESP8266WiFi.h>

#define SSID "what"
#define PASSWORD "arewedoing"     
#define PORT 33000              // The port can be any number if high enough
WiFiServer server(PORT);        // Create a server object that listens on the specified port
//===========================================//

// loop vars
uint32_t sensor_interval, last_cycle;


void echo_isr1(){  
  switch(digitalRead(sensor1._echo)){
    case HIGH:
      sensor1._start=micros();
      break;
    case LOW:
      sensor1._end=micros();
      sensor1._finished=true;
      break;
  }   
}

void echo_isr2(){  
  switch(digitalRead(sensor2._echo)){
    case HIGH:
      sensor2._start=micros();
      break;
    case LOW:
      sensor2._end=micros();
      sensor2._finished=true;
      break;
  }   
}

void echo_isr3(){  
  switch(digitalRead(sensor3._echo)){
    case HIGH:
      sensor3._start=micros();
      break;
    case LOW:
      sensor3._end=micros();
      sensor3._finished=true;
      break;
  }   
}


void setup() {
  // put your setup code here, to run once:
  // Setup serial coms
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  //start ultrasonic sensors
  sensor1.begin();
  attachInterrupt(sensor1._int, echo_isr1, CHANGE);
  sensor2.begin();
  attachInterrupt(sensor2._int, echo_isr2, CHANGE);
  sensor3.begin();
  attachInterrupt(sensor3._int, echo_isr3, CHANGE);
  
  sensor1.start();
  sensor2.start();
  sensor3.start();

  // Set Rasp as an Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, PASSWORD);

  // Start the server
  delay(2000);
  server.begin();
  Serial.println("Server started");

  //loop vars
  /* sensor_interval = 10;
  last_cycle = 0; */
  now=millis();
}


void loop() {
  // put your main code here, to run repeatedly:
  /*digitalWrite(LED_BUILTIN, HIGH);
  uint32_t now = millis();
  uint32_t delta = now - last_cycle;

  if (delta >= sensor_interval) {
    last_cycle = now;

    Serial.println(millis());

  delay(1000);*/
  if((millis()-now)>LOOP_TIME){
    now=millis();
    Serial.println(vel_L);
    robot.setMotorPWM(0,100);
    robot.setMotorPWM(1,100);
  

    //Read distance sensors if ready
    if(sensor1.isFinished()){
      dist1 = sensor1.getRange();
      sensor1.start();
    }
    if(sensor2.isFinished()){
      dist2 = sensor2.getRange();
      sensor2.start();
    }
    if(sensor3.isFinished()){
      dist3 = sensor3.getRange();
      sensor3.start();
    }

    Serial.print("Sensor 1: ");
    Serial.print(dist1);
    Serial.print("; Sensor 2: ");
    Serial.print(dist2);
    Serial.print("; Sensor 3: ");
    Serial.println(dist3);
  

    // Check for incoming client connections
    WiFiClient client = server.available();
    if (client) {
        //Serial.println("Client connected");
        while (client.connected()) {
            // Receive data from the client while he is available
            if (client.available()) {
                // Read the data from the client until an ending character is found
                String request = client.readStringUntil('\0');
                Serial.print("Received: ");
                Serial.print("[PC] "); 
                Serial.println(request);
                // Send a response to the client
                String response = "ACK - \"" + request + "\"";
                Serial.print("[RASP] ");
                client.print(response);
                // Sends possible remaining data to the client, cleaning the buffer
                client.flush(); 
            }
        }
    }
  }
}
