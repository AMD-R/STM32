#define __STM32F1__
#define USE_USBCON
#include "ArmController.h"
#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>
#include "BasicStepperDriver.h"
#include "A4988.h"
#include "MultiDriver.h"

/* Message definition
  ind 0 - Command 
      1 - RPM
      2 - Steps
      
  Commands
  0 - Move
  1 - Home
*/

uint32_t position_Z = 0;
uint32_t position_Y = 0;
bool stop_x = 0;
bool move_x = 0;
bool DEBUG_LED = 0;
long move_start_time;
  
ros::NodeHandle  nh;
BasicStepperDriver stepper_Z(MOTOR_STEPS_Z, PIN_DIR_Z, PIN_STEP_Z);
A4988 stepper_Y(MOTOR_STEPS_Y, PIN_DIR_Y, PIN_STEP_Y, PIN_MS1_Y, PIN_MS2_Y, PIN_MS3_Y);
MultiDriver stepper_multi(stepper_Y, stepper_Z);

void callback_stepper(const std_msgs::Int16MultiArray& msg){
  digitalWrite(PC13, !DEBUG_LED);
  DEBUG_LED = !DEBUG_LED;
  if (msg.data[INDEX_CMD] == COMMAND_HOME)
  {
    // TODO - REPLY STATUS
    bool resp_z = homeMotor_Z();
    bool resp_y = homeMotor_Y();
  } 
  else if(msg.data[INDEX_CMD] == COMMAND_MOVE)
  {
    if (stepper_multi.nextAction() != 0)
    {
      nh.loginfo("Arm still moving! Command ignored");
      return;
    }
    int rpm_y =  msg.data[INDEX_RPM_Y];
    int steps_y =  msg.data[INDEX_STEPS_Y];
    int rpm_z =  msg.data[INDEX_RPM_Z];
    int steps_z =  msg.data[INDEX_STEPS_Z];
    if (rpm_y <= 0 || rpm_z <= 0 
        || position_Z + steps_z > STEP_LIM_Z  || position_Y + steps_y > STEP_LIM_Y 
        || position_Z + steps_z < 0           || position_Y + steps_y < 0)
    {
      nh.loginfo("Sanity Check failed!");
    }
    else
    {
      stepper_Y.setRPM(rpm_y);
      stepper_Z.setRPM(rpm_z);
      stepper_multi.startMove(steps_y * MICROSTEPS_Y, steps_z * MICROSTEPS_Z);
      position_Z += steps_z;
      position_Y += steps_y;
      // TODO - REPLY STATUS
    }
  }
}

void callback_X(const std_msgs::Bool& msg){
  move_x = 1;
  move_start_time = millis();
  if (msg.data == COMMAND_EXTEND)
  {
    digitalWrite(PIN_DIR_X, LOW);
    digitalWrite(PIN_PWM_X, HIGH);
  } 
  else
  {
    digitalWrite(PIN_DIR_X, HIGH);
    digitalWrite(PIN_PWM_X, HIGH);
  }
}

void callback_stop_X(const std_msgs::Empty& msg){
  stop_x = 1;
}

bool homeMotor_Z()
{
  stepper_Z.move(MOTOR_STEPS_Z * MICROSTEPS_Z * 5);
  stepper_Z.startMove(-100 * MOTOR_STEPS_Z * MICROSTEPS_Z);
  while(stepper_Z.nextAction() != 0 )
  {
    if(!digitalRead(PIN_LIM_Z))
    {
      stepper_Z.stop();
      position_Z = 0;
      return true;
      break;
    }
  }
  return false;
}

bool homeMotor_Y()
{
  stepper_Y.move(MOTOR_STEPS_Y * MICROSTEPS_Y * 5);
  stepper_Y.startMove(-100 * MOTOR_STEPS_Y * MICROSTEPS_Y);
  while(stepper_Y.nextAction() != 0 )
  {
    if(!digitalRead(PIN_LIM_Y))
    {
      stepper_Y.stop();
      position_Y = 0;
      return true;
      break;
    }
  }
  return false;
}

ros::Subscriber<std_msgs::Int16MultiArray> sub_stepper(TOPIC_STEPPER_COMMAND, &callback_stepper);
ros::Subscriber<std_msgs::Bool> sub_X(TOPIC_COMMAND_X, &callback_X);
ros::Subscriber<std_msgs::Empty> sub_stop_X(TOPIC_STOP_X, &callback_stop_X);

void setup() { 
  stepper_Z.begin(BASE_RPM_Z, MICROSTEPS_Z);
  stepper_Y.begin(BASE_RPM_Y, MICROSTEPS_Y);
  stepper_Z.setPulseState(LOW);
  
  pinMode(PIN_LIM_Z, INPUT_PULLUP);
  pinMode(PIN_LIM_Y, INPUT_PULLUP);
  pinMode(PIN_PWM_X, OUTPUT);
  pinMode(PIN_DIR_X, OUTPUT);
  pinMode(PC13, OUTPUT);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  
  nh.subscribe(sub_stepper);
  nh.subscribe(sub_X);
  nh.subscribe(sub_stop_X);

  homeMotor_Z();
  homeMotor_Y();
  
  move_x = 1;
  move_start_time = millis();
  digitalWrite(PIN_DIR_X, HIGH);
  digitalWrite(PIN_PWM_X, HIGH);
}

void loop() {  
  nh.spinOnce();
  stepper_multi.nextAction();
  if (move_x && (millis() - move_start_time > TIME_LIM_X || stop_x)) // TODO : Current sensing
  {
    digitalWrite(PIN_PWM_X, LOW);
    stop_x = 0;
    move_x = 0;
  }
  //delay(2);
}
