#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

int16_t gyroY, gyroRate;
float gyroAngle=0;
unsigned long currTime, prevTime=0, loopTime;

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
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  
  gyroY = mpu.getRotationY();
  gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroRate*loopTime/1000;
  
  Serial.println(gyroAngle);
  //face down = 90 deg
  //face up = -90 deg
  //balanced = 0 deg
}
