#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

MPU6050 mpu;

int16_t accX, accZ;
float accAngle;

void setup() {
  mpu.initialize();
  Serial.begin(9600);
  //  mpu.setYAccelOffset(1593);
  mpu.setXAccelOffset(-4544);
  mpu.setZAccelOffset(1549);
  mpu.setYGyroOffset(-28);
}

void loop() {
  accZ = mpu.getAccelerationZ();
  //accY = mpu.getAccelerationY();
  accX = mpu.getAccelerationX();

  accAngle = atan2(accX, accZ) * RAD_TO_DEG;

  if (isnan(accAngle));
  else {
    Serial.println(accAngle);
    //Serial.println(accZ);
    //Serial.println(accX);
  }
}
