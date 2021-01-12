#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

MPU6050 mpu;

int16_t accY, accZ, accX;
float accAngle;

void setup() {  
  mpu.initialize();
  Serial.begin(9600);
  mpu.setXAccelOffset(-4899);
  mpu.setYAccelOffset(-1674);
  mpu.setZAccelOffset(1484);
  mpu.setXGyroOffset(68);
  mpu.setYGyroOffset(-34);
  mpu.setZGyroOffset(15);  
}

void loop() {  
  accX = mpu.getAccelerationX();
  accZ = mpu.getAccelerationZ();
  accY = mpu.getAccelerationY();
  //Serial.println("----------");
  //Serial.println(accZ);
  //Serial.println(accY);
  //Serial.println(accX);
  //Serial.println("----------");
  accAngle = -atan2(accX, accZ)*RAD_TO_DEG;
  
  if(isnan(accAngle));
  else
    Serial.print("Ang: ");
    Serial.println(accAngle);
    //face down = 90 deg
    //face up = -90 deg
    //balanced = 0 deg
}
