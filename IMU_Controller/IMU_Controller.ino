#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

#define INTERRUPT_PIN PA10  // use pin 2 on Arduino Uno & most boards
#define LED_PIN PB13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[28] = { '$', 0x03, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

bool DEBUG_LED;

void setup() {
    pinMode (INTERRUPT_PIN, INPUT);
    
    Wire.begin();
    Wire.setClock(400000);
    
    Serial.begin(115200);
    Serial.println(F("Initializing I2C devices..."));
    
    mpu.initialize();
    mpu.testConnection();
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);
  
      attachInterrupt(digitalPinToInterrupt (INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
  
      dmpReady = true;
  
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      while (1)
      {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(250);
      }
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    if (!dmpReady)
    {
      Serial.println("dmpReady Failed!");
      digitalWrite(LED_BUILTIN, !DEBUG_LED);
      DEBUG_LED = !DEBUG_LED;
      return;
    }
    
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    // gyro values
    teapotPacket[10] = fifoBuffer[16];
    teapotPacket[11] = fifoBuffer[17];
    teapotPacket[12] = fifoBuffer[20];
    teapotPacket[13] = fifoBuffer[21];
    teapotPacket[14] = fifoBuffer[24];
    teapotPacket[15] = fifoBuffer[25];
    // accelerometer values
    teapotPacket[16] = fifoBuffer[28];
    teapotPacket[17] = fifoBuffer[29];
    teapotPacket[18] = fifoBuffer[32];
    teapotPacket[19] = fifoBuffer[33];
    teapotPacket[20] = fifoBuffer[36];
    teapotPacket[21] = fifoBuffer[37];
    //temperature
    int16_t temperature = mpu.getTemperature();
    teapotPacket[22] = temperature >> 8;
    teapotPacket[23] = temperature & 0xFF;
    Serial.write(teapotPacket, 28);
    teapotPacket[25]++; // packetCount, loops at 0xFF on purpose
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
  }
}
