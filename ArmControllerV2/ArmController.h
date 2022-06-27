// STEPPER PROTOCALL //
#define COMMAND_MOVE 0
#define COMMAND_HOME 1
#define INDEX_CMD 0
#define INDEX_RPM_Y 1
#define INDEX_STEPS_Y 2
#define INDEX_RPM_Z 3
#define INDEX_STEPS_Z 4

// LINEAR PROTOCALL //
#define COMMAND_EXTEND 1
#define COMMAND_RETRACT 0

#define TOPIC_NAMESPACE "/arm"
#define TOPIC_MOTOR_X TOPIC_NAMESPACE "/motor_x"
#define TOPIC_COMMAND_X TOPIC_MOTOR_X "/motor_cmd"
#define TOPIC_STOP_X TOPIC_MOTOR_X "/motor_stop"
#define TOPIC_MOTOR_STEPPER TOPIC_NAMESPACE "/stepper"
#define TOPIC_STEPPER_COMMAND TOPIC_MOTOR_STEPPER "/motor_cmd"

// Z MOTOR PARAMS // 
#define MOTOR_STEPS_Z 200
#define BASE_RPM_Z 240
#define MICROSTEPS_Z 8
#define M_TO_STEPS_Z 100000
#define STEP_LIM_Z M_TO_STEPS_Z * MICROSTEPS_Z * 0.10 // in Meters
#define PIN_STEP_Z PB12
#define PIN_DIR_Z PB13
#define PIN_ENA_Z 10
#define PIN_LIM_Z PB14 // 10k-PULLUP

// X MOTOR PARAMS // 
#define TIME_LIM_X 20*1000
#define PIN_PWM_X PA8
#define PIN_DIR_X PA9

// Y MOTOR PARAMS // 
#define MOTOR_STEPS_Y 20
#define BASE_RPM_Y 120
#define MICROSTEPS_Y 4
#define M_TO_STEPS_Y 40000
#define STEP_LIM_Y M_TO_STEPS_Y * MICROSTEPS_Y * 8
#define PIN_STEP_Y PB4
#define PIN_DIR_Y PB5
#define PIN_ENA_Y 10
#define PIN_MS1_Y PB6
#define PIN_MS2_Y PB7
#define PIN_MS3_Y PB8
#define PIN_LIM_Y PB9 // 10k-PULLUP

/* Message definition
  ind 0 - Command 
      1 - RPM
      2 - Steps
      
  Commands
  0 - Move
  1 - Home
*/
