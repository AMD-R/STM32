#define __STM32F1__
#define USE_USBCON
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>

#define MOTOR_1 0
#define MOTOR_2 1
#define BRAKE 0
#define CW    1
#define CCW   2

bool DEBUG_LED;

ros::NodeHandle  nh;
std_msgs::Int16 encoder1;
std_msgs::Int16 encoder2;

ros::Publisher encoderValue1("rwheel", &encoder1);
ros::Publisher encoderValue2("lwheel", &encoder2);

#define MOTOR_2_DIR_PIN PB8 //prev 14
#define MOTOR_1_DIR_PIN PB9 //prev 15
#define MOTOR_2_PWM_PIN PB0
#define MOTOR_1_PWM_PIN PB1

const int encoderPin2A = PB12; //prev 6
const int encoderPin2B = PB13; //prev 7
const int encoderPin1A = PB14; //prev 8
const int encoderPin1B = PB15; //prev 9
const int MPU_INTERRUPT = PA10;

volatile int16_t currentPosition1 = 0;
volatile int16_t currentPosition2 = 0;

uint32_t prev_time;

void pwm_1_callback( const std_msgs::Float32& pwm_value) {
  float pwm = 0;
  pwm = pwm_value.data;

  if ( pwm > 0 )
  {
    driveMotor(MOTOR_1_PWM_PIN, MOTOR_1_DIR_PIN, CCW, pwm);
  }
  else
  {
    driveMotor(MOTOR_1_PWM_PIN, MOTOR_1_DIR_PIN, CW, abs(pwm));
  }
}

void pwm_2_callback( const std_msgs::Float32& pwm_value) {
  float pwm = 0;
  pwm = pwm_value.data;

  if ( pwm > 0 )
  {
    driveMotor(MOTOR_2_PWM_PIN, MOTOR_2_DIR_PIN, CW, pwm);
  }
  else
  {
    driveMotor(MOTOR_2_PWM_PIN, MOTOR_2_DIR_PIN, CCW, abs(pwm));
  }
}

ros::Subscriber<std_msgs::Float32> pwm1_sub("motor_cmd_r", &pwm_1_callback);
ros::Subscriber<std_msgs::Float32> pwm2_sub("motor_cmd_l", &pwm_2_callback);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // PIN SETUP //
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode (MOTOR_1_DIR_PIN, OUTPUT);
  pinMode (MOTOR_2_DIR_PIN, OUTPUT);
  pinMode (MOTOR_1_PWM_PIN, OUTPUT);
  pinMode (MOTOR_2_PWM_PIN, OUTPUT);
  pinMode (encoderPin1A, INPUT_PULLUP);
  pinMode (encoderPin1B, INPUT_PULLUP);
  pinMode (encoderPin2A, INPUT_PULLUP);
  pinMode (encoderPin2B, INPUT_PULLUP);

  attachInterrupt (digitalPinToInterrupt (encoderPin1A), readEncoder1A, CHANGE);
  attachInterrupt (digitalPinToInterrupt (encoderPin1B), readEncoder1B, CHANGE);
  attachInterrupt (digitalPinToInterrupt (encoderPin2A), readEncoder2A, CHANGE);
  attachInterrupt (digitalPinToInterrupt (encoderPin2B), readEncoder2B, CHANGE);
  analogWriteFrequency(20000);
  digitalWrite(MOTOR_1_PWM_PIN, LOW);
  digitalWrite(MOTOR_2_PWM_PIN, LOW);

  //  ROS SETUP  //
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(encoderValue1);
  nh.advertise(encoderValue2);
  nh.subscribe(pwm1_sub);
  nh.subscribe(pwm2_sub);

  prev_time = millis();
}


void driveMotor(int16_t motor_pwm_pin, int16_t motor_dir_pin, int16_t direct, uint16_t pwm)
{
  if (direct == CW)
  {
    digitalWrite(motor_dir_pin, LOW);
  }
  else if (direct == CCW)
  {
    digitalWrite(motor_dir_pin, HIGH);
  }
  analogWrite(motor_pwm_pin, pwm);
}

void readEncoder1A()
{
  if (digitalRead(encoderPin1A) != digitalRead(encoderPin1B))
  {
    currentPosition1++;
  }
  else
  {
    currentPosition1--;
  }
}

void readEncoder1B()
{
  if (digitalRead(encoderPin1A) == digitalRead(encoderPin1B))
  {
    currentPosition1++;
  }
  else
  {
    currentPosition1--;
  }
}

void readEncoder2A()
{
  if (digitalRead(encoderPin2A) != digitalRead(encoderPin2B))
  {
    currentPosition2++;
  }
  else
  {
    currentPosition2--;
  }
}

void readEncoder2B()
{
  if (digitalRead(encoderPin2A) == digitalRead(encoderPin2B))
  {
    currentPosition2++;
  }
  else
  {
    currentPosition2--;
  }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  
  encoder1.data = currentPosition1;
  encoder2.data = currentPosition2;
  encoderValue1.publish( &encoder1 );
  encoderValue2.publish( &encoder2 );

  nh.spinOnce();
  
  while (millis() - prev_time < 20) {yield();nh.spinOnce();}
  prev_time = millis();
  //display_mallinfo();
  //}
  
}
