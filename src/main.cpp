#include <Arduino.h>
#include <Robot.h>
#include <HC_SR04.h>

volatile int count_R = 0;// count for right motor encoder
volatile int count_L = 0;// count for left motor encoder
volatile float vel_R = 0;
volatile float vel_L = 0;

unsigned long now, now1;

unsigned long LOOP_TIME = 37;

Robot robot;

volatile bool connected = false;

//============ ULTRASONIC sensor =============//
HC_SR04 sensor1(Ultrasonic_TRIG1, Ultrasonic_ECHO1, digitalPinToInterrupt(Ultrasonic_ECHO1));
HC_SR04 sensor2(Ultrasonic_TRIG2, Ultrasonic_ECHO2, digitalPinToInterrupt(Ultrasonic_ECHO2));
HC_SR04 sensor3(Ultrasonic_TRIG3, Ultrasonic_ECHO3, digitalPinToInterrupt(Ultrasonic_ECHO3));
//===========================================//

//================= WIFI ====================//
#include <ESP8266WiFi.h>

#define SSID "what"
#define PASSWORD "arewedoing"     
#define PORT 33000              // The port can be any number if high enough
WiFiServer server(PORT);        // Create a server object that listens on the specified port
WiFiClient client;
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

void setup1(){
  //Serial.begin(115200);
  // Set Rasp as an Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, PASSWORD);
  Serial.println("Hi");
  // Start the server
  delay(2000);
  
  Serial.println("Hi");
  server.begin();
  Serial.println("Hi");
  Serial.println("Server started");
  now1=millis();
  while(!client)
    client = server.available();
  connected = true;
}

void loop1(){
  // Check for incoming client connections
  if(!client.connected())
    client = server.available();
  if(!client)
    client = server.available();
  if((millis()-now1)>LOOP_TIME){
    now1=millis();
    
    if (client) {
        Serial.println("Client connected");
        while(client.connected()) {
            if((millis()-now1)>LOOP_TIME){
              now1=millis();
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
                  Serial.println("Hi");
                  client.print(vel_L);
                  client.print(vel_R);
                  client.print("\0");
                  client.flush();
              } 
          }
        }

    }
  }
}


void setup() {
  // put your setup code here, to run once:
  // Setup serial coms
  //set_sys_clock_khz(250000, true);
  Serial.begin(115200);
  //while(!Serial);


  robot.init();

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

  //loop vars
  /* sensor_interval = 10;
  last_cycle = 0; */
  //while(!connected);
  
  now=millis();
}


void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH);
  /*uint32_t now = millis();
  uint32_t delta = now - last_cycle;

  if (delta >= sensor_interval) {
    last_cycle = now;

    Serial.println(millis());

  delay(1000);*/
  //Serial.println(millis()-now);
  if((millis()-now)>LOOP_TIME){
    now=millis();
    Serial.print(count_L);
    Serial.print(" ");
    Serial.println(count_R);
    robot.setMotorPWM(0,100);
    robot.setMotorPWM(1,100);
  

    //Read distance sensors if ready
    if(sensor1.isFinished()){
      robot.sonic_dist[0] = sensor1.getRange();
      sensor1.start();
    }
    if(sensor2.isFinished()){
      robot.sonic_dist[1] = sensor2.getRange();
      sensor2.start();
    }
    if(sensor3.isFinished()){
      robot.sonic_dist[2] = sensor3.getRange();
      sensor3.start();
    }

    /* Serial.print("Sensor 1: ");
    Serial.print(dist1);
    Serial.print("; Sensor 2: ");
    Serial.print(dist2);
    Serial.print("; Sensor 3: ");
    Serial.println(dist3); */

  }
}
