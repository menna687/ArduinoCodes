//applying kalman filter to imu readings

#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include "I2Cdev.h"
#define INTERRUPT_PIN 2

MPU6050 mpu;

bool dmpReady = false;  
uint16_t fifoCount;  
uint16_t packetSize;
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint8_t fifoBuffer[64];
 
volatile bool mpuInterrupt = false;

VectorFloat gravity;
Quaternion q;
float ypr[3]; 

int predState, state, R, Q, sensorReading, predUncert, uncert, KG;     //KF variables

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  while(!Serial);
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  mpu.testConnection();
  devStatus = mpu.dmpInitialize();
  mpu.setZGyroOffset(-49);
  
  if (devStatus == 0) {
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
  }
 // assumed values
  state=30;
  uncert=10;
  Q=5;
  R=3;  //depending on sensor
}

void loop() {
   if (dmpReady){
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){     
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.println(ypr[0] * 180/M_PI);

      sensorReading=ypr[0] * 180/M_PI;
  
     //update predictions to be previous loop outputs (or initialized values)
     predState = state; 
     predUncert = uncert + Q;
  
     //get Kalman gain  
     KG = predUncert/(predUncert +R);
  
     //new state value   
     state = predState+KG*(sensorReading-predState); 
     Serial.println(state);
  
     //new uncertainity   
     uncert = (1-KG)*predUncert; 
    }
   }  
}

void dmpDataReady() {
mpuInterrupt = true;
}
