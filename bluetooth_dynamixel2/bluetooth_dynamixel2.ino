/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
Servo myservo2;
// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
 


const uint8_t DXL_ID[3] = {1, 2, 3};
const float DXL_PROTOCOL_VERSION = 2.0;
//int initial_angle[3] = {900, 447, 4700};
int initial_angle[3] = {200, 50, 530};//{176, 59, 192}; //{249, 59, 570};
int end_angle[3] = {140, 50, 180};
int m_angle[3] = {341, 59, 283};
int check_angle[3] = {-30, 150, 60};
String msg, mode_select, motor_dir, servo1_dir, servo2_dir, isawake;
int motor_select;
boolean is_position = 1;
int sleep_mode;
int first, second, third, fourth, len;
int moving_average[7];
//int moving_average[20];

int cnt = 0, sleep_cnt;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void position_setup(){
  for(char i=0; i<3; i++){
    dxl.ping(DXL_ID[i]);
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_POSITION);
    dxl.torqueOn(DXL_ID[i]);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[i], 30);
  }
}

void velocity_setup(){
  for(char i=0; i<3; i++){
    dxl.ping(DXL_ID[i]);
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_VELOCITY);
    dxl.torqueOn(DXL_ID[i]);
  }
}

String get_angle(){
  for(int i=0; i<3; i++){
    m_angle[i] = dxl.getPresentPosition(DXL_ID[i]);
  }
  return String(m_angle[0]) + " " + String(m_angle[1]) + " " + String(m_angle[2]);
}

void VMDforDistance(int motor_select, String motor_dir){
  int dir = motor_dir.equals("c")?1:(motor_dir.equals("C")?-1:0);
  int current_angle;
  switch(motor_select){
      case 1:                             //XL-1
        dxl.setGoalVelocity(DXL_ID[1], 0);
        dxl.setGoalVelocity(DXL_ID[2], 0);
        current_angle = dxl.getPresentPosition(DXL_ID[0], UNIT_DEGREE);
        if (abs(current_angle)< 300000) dxl.setGoalVelocity(DXL_ID[0], dir*20);
        else {
          dxl.setGoalVelocity(DXL_ID[0], 0);
          Serial1.println("361 361 361 361");
        }
        break;
      case 2:                            //Mx-2
        dxl.setGoalVelocity(DXL_ID[0], 0);
        dxl.setGoalVelocity(DXL_ID[2], 0);
        current_angle = dxl.getPresentPosition(DXL_ID[1], UNIT_DEGREE);
        if (abs(current_angle)< 300000) dxl.setGoalVelocity(DXL_ID[1], dir*20);
        else {
          dxl.setGoalVelocity(DXL_ID[1], 0);
          Serial1.println("361 361 361 361");
        }
        break;
      case 3:                             //XL-3
        dxl.setGoalVelocity(DXL_ID[0], 0);
        dxl.setGoalVelocity(DXL_ID[1], 0);
        current_angle = dxl.getPresentPosition(DXL_ID[2], UNIT_DEGREE);
        if (abs(current_angle)< 300000) dxl.setGoalVelocity(DXL_ID[2], dir*20);
        else {
          dxl.setGoalVelocity(DXL_ID[2], 0);
          Serial1.println("361 361 361 361");
        }
        break; 
      case 4:
        myservo2.write(95 - dir*20);
        break;
    }
}

void VMDforTracking(int motor_select, String motor_dir, String servo1_dir){
  int velocity = motor_dir.toInt();
  int average_velocity = 0;
  if (velocity == 1000){
    velocity = 0;
    dxl.setGoalVelocity(DXL_ID[1], velocity); 
  }
  else{
    moving_average[cnt] = velocity * 2;
    cnt++;
    if (cnt > 6)cnt = 0;
    int max_value = moving_average[0], min_value = moving_average[0];
    for (char i = 0; i<7; i++){
      average_velocity += moving_average[i];
    }
    for (char i = 1; i< 7; i++){
      if (max_value < moving_average[i]) max_value = moving_average[i];
      if (min_value > moving_average[i]) min_value = moving_average[i];
    }
    average_velocity -= max_value;
    average_velocity -= min_value;
    average_velocity /= 5;
    
    if (average_velocity < 3 && average_velocity > -3) {
      moving_average[cnt] = 0; 
      dxl.setGoalVelocity(DXL_ID[1], 0);
      if (servo1_dir.equals("0")) myservo.write(90);
      else if (servo1_dir.equals("2")) myservo.write(80);
      else if (servo1_dir.equals("1")) myservo.write(108);
    }
    else if (average_velocity < 40 && average_velocity > -40){
      dxl.setGoalVelocity(DXL_ID[1], average_velocity); 
      myservo.write(90);
    }
  }
}

void Initial_angle(){
  int current_angle[3];
  char arrive[3] = {0, 0, 0};
  for(char i=0; i<3; i++){
    current_angle[i] = dxl.getPresentPosition(DXL_ID[i], UNIT_DEGREE);
    if(i == 0) dxl.setGoalVelocity(DXL_ID[i], current_angle[i]>initial_angle[i]? -40:40);
    else if(i==1)dxl.setGoalVelocity(DXL_ID[i], current_angle[i]>initial_angle[i]?-40:40);  
    else dxl.setGoalVelocity(DXL_ID[i], current_angle[i]>initial_angle[i]?-40:40);  
  }
  while (1){
    for(char i=0; i<3; i++){
      current_angle[i] = dxl.getPresentPosition(DXL_ID[i], UNIT_DEGREE);
      if (abs(current_angle[i] - initial_angle[i]) < 5){              
        dxl.setGoalVelocity(DXL_ID[i], 0);
        arrive[i] = 1;
      }
    }
    if (arrive[0] == 1 && arrive[1] == 1 && arrive[2] == 1){
      break;
    }
  }
}

void terminate_angle(){
  int current_angle[3];
  char arrive[3] = {0, 0, 0};
  for(char i=0; i<3; i++){
    current_angle[i] = dxl.getPresentPosition(DXL_ID[i], UNIT_DEGREE);
    if(i == 0) dxl.setGoalVelocity(DXL_ID[i], current_angle[i]>end_angle[i]? -40:40);
    else if(i==1)dxl.setGoalVelocity(DXL_ID[i], current_angle[i]>end_angle[i]?-40:40);  
    else dxl.setGoalVelocity(DXL_ID[i], current_angle[i]>end_angle[i]?-40:40);  
  }
  while (1){
    for(char i=0; i<3; i++){
      current_angle[i] = dxl.getPresentPosition(DXL_ID[i], UNIT_DEGREE);
      if (abs(current_angle[i] - end_angle[i]) < 5){              
        dxl.setGoalVelocity(DXL_ID[i], 0);
        arrive[i] = 1;
      }
    }
    if (arrive[0] == 1 && arrive[1] == 1 && arrive[2] == 1){
      break;
    }
  }
}

void Check(){
  int angle = dxl.getPresentPosition(DXL_ID[1], UNIT_DEGREE);
  check_angle[2] = angle;
  dxl.setGoalVelocity(DXL_ID[1], angle>check_angle[0]?-30:30);
  while(1){
    angle = dxl.getPresentPosition(DXL_ID[1], UNIT_DEGREE);
    if (abs(angle - check_angle[0]) < 2){              
          dxl.setGoalVelocity(DXL_ID[1], 0);
          break;
    }
  }
  delay(500);
  dxl.setGoalVelocity(DXL_ID[1], angle>check_angle[1]?-30:30);
  while(1){
    angle = dxl.getPresentPosition(DXL_ID[1], UNIT_DEGREE);
    if (abs(angle - check_angle[1]) < 2){              
          dxl.setGoalVelocity(DXL_ID[1], 0);
          break;
    }
  }
  delay(500);
  dxl.setGoalVelocity(DXL_ID[1], angle>check_angle[2]?-30:30);
  while(1){
    angle = dxl.getPresentPosition(DXL_ID[1], UNIT_DEGREE);
    if (abs(angle - check_angle[2]) < 2){              
          dxl.setGoalVelocity(DXL_ID[1], 0);
          break;
    }
  }
  delay(500);
}


void setup() {
  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  Serial1.begin(115200);
  velocity_setup();
//  Initial_angle();
  terminate_angle();
  myservo.attach(30);
  myservo2.attach(31);
  sleep_cnt = 0;
  sleep_mode = -1;
}



void loop() {
  while(Serial1.available()) {
    msg = Serial1.readStringUntil('\n');
    msg.trim();
    first = msg.indexOf(" ");
    second = msg.indexOf(" ", first+1);
    third = msg.indexOf(" ", second+1);
    fourth = msg.indexOf(" ", third+1);
    len = msg.length();

    mode_select = msg.substring(0, first);
    motor_select = msg.substring(first+1, second).toInt();
    motor_dir = msg.substring(second+1, third);
    servo1_dir = msg.substring(third+1, fourth);
    isawake = msg.substring(fourth+1, len);
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (sleep_mode == -1){
    if (mode_select.equals("start")){
      Initial_angle();
      sleep_mode = 0;
    }
//    Initial_angle();
//    sleep_mode = 0;
  }
  else if (sleep_mode == 0){
    if (isawake.equals("0")) sleep_cnt++;
    else if (isawake.equals("1")) sleep_cnt = 0;
    if (sleep_cnt == 500){
      sleep_mode = 1;
    }
    if(mode_select.equals("Distance")){
      VMDforDistance(motor_select, motor_dir);
      if (motor_select == 0){
        msg = get_angle();
        mode_select = "";
      }
    }   
    else if(mode_select.equals("Test")){
      Check();
      mode_select = "";
    }   
    else if(mode_select.equals("Tracking")){
      delay(5);
      VMDforTracking(motor_select, motor_dir, servo1_dir);
    }
  }
  else if (sleep_mode == 1){
    terminate_angle();
    sleep_mode = -1;
  }  
}
    
      
