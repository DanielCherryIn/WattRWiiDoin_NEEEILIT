#include <Arduino.h>
#include <Robot.h>
#include <HC_SR04.h>

volatile int count_R = 0;// count for right motor encoder
volatile int count_L = 0;// count for left motor encoder
volatile float vel_R = 0;
volatile float vel_L = 0;

unsigned long now, now1, node_timer;

unsigned long LOOP_TIME = 37, NODE = 500;

Robot robot;

volatile bool connected = false;

bool walk = false;

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
  //delay(2000);
  Serial.println("not stop");
  now=millis();
  node_timer=millis();
}
direction_t *v;
int v_index = 0;
int v_size = 0;

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH);

  if((millis()-now)>LOOP_TIME){
    now=millis();
    
    /*Serial.print(robot.mot[0].vel);
    Serial.print(" ");
    Serial.print(robot.mot[1].vel);
    Serial.print(" ");
    Serial.print(robot.mot[0].pwm);
    Serial.print(" ");
    Serial.print(robot.mot[1].pwm);
    Serial.print(" ");
    Serial.print(robot.sonic_dist[0]);
    Serial.print(" ;");
    Serial.print(robot.sonic_dist[1]);
    Serial.print(" ;");
    Serial.print(robot.sonic_dist[2]);
    Serial.print(" ");
    Serial.print(robot.wall_F);
    Serial.print(" ");
    Serial.print(robot.wall_L);
    Serial.print(" ");
    Serial.println(robot.wall_R);*/

    /* robot.setMotorPWM(0,100);
    robot.setMotorPWM(1,100); */

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

    if (robot.sonic_dist[0] < robot.wall_thresh)
      robot.wall_R=true;
    else
      robot.wall_R=false;

    if (robot.sonic_dist[1] < robot.wall_thresh)
      robot.wall_L=true;
    else
      robot.wall_L=false;

    if (robot.sonic_dist[2] < 15)
    {
      robot.stop();
      walk = false;
      robot.wall_F=true;
    }
    else
      robot.wall_F=false;

    
    Serial.print("V size ");
    Serial.print(v_size);
    Serial.print(" Stack size ");
    Serial.print(stack.size());
    Serial.print(" walk ");
    Serial.print(walk);
    Serial.print(" cur node id ");
    Serial.print(robot.cur->id);
    Serial.print(" robot dir ");
    Serial.println(robot.dir);
    

    if(v_size == 0 && !walk)
    {
      bool neighbours[4] = {!robot.wall_F, !robot.wall_R, 0, !robot.wall_L};


      robot.stop();
      delay(250);
      build(robot.cur, robot.dir, neighbours);   
        node_timer=millis();
      Serial.print(" Stack size ");
      Serial.print(stack.size());  
      Serial.print(" Stack top ");
      Serial.print((long long)stack.front());
          //Serial.print(" robot dir ");
    //Serial.println(robot.dir);
      //node* next_node = explore();
      //Serial.print(" node addr ");
      //if(next_node != NULL) Serial.println((long long)(next_node));
      //else Serial.println("NULL node");
      
      node* next_node = explore();
      
      //Serial.print(" Stack size ");
      //Serial.print(stack.size());  
      //Serial.print(" Stack top ");
      //Serial.print((long long)stack.front());
      Serial.print(" node addr ");
      Serial.println((long long)(next_node));
          //Serial.print(" robot dir ");
    //Serial.println(robot.dir);
      if(next_node == NULL)
      {
        while(next_node == NULL)
        {
          build(robot.cur, robot.dir, neighbours); 
          next_node = explore();
        }
        //finish mapping
        Serial.println("MAPING done!");
        //walk = false;
              //robot.stop();
        
      }
      else
      {
        Serial.println("Gen path");
        v = gen_next_path(robot.cur, robot.dir, next_node, &v_size);
        for(int i = 0; i < v_size; i++) Serial.print(v[i]);
        Serial.print(" robot dir ");
    //Serial.println(robot.dir);
        v_index = 0;

        walk = true;
        node_timer=millis();
      }
    }
   //Serial.print("stacks ");
    //Serial.print(v_index);
    //Serial.println(v_size);
    if(walk)
    {
      robot.followWall(60, 140, 30, (float) (LOOP_TIME/1000));
      if((millis()-node_timer>NODE)){
        walk = false;
        robot.stop();
      }
    }
    else if(v_index < v_size)
    {
      direction_t step = v[v_index++];
      robot.sum_error=0;
      switch(step)
      {
        case front:
          walk = true;
          node_timer=millis();
        break;
        case left:
          robot.dir = (direction_t)((robot.dir + 3) % 4); 
          robot.spin_left(80);
        break;
        case right:
          robot.dir = (direction_t)((robot.dir + 1) % 4); 
          robot.spin_right(80);
        break;
        case back:
          robot.dir = (direction_t)((robot.dir + 2) % 4); 
          robot.spin_right(80);
          robot.spin_right(80);
        break;
      }
    }
    else
    {
      v_size = 0;
      v_index = 0;
    }
  }
}

