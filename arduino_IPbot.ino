#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include <NewPing.h>

#define leftMotorPWMPin   6
#define leftMotorDirPin   7
#define rightMotorPWMPin  5
#define rightMotorDirPin  4
#define test 10
#define test2 11

#define TRIGGER_PIN 9
#define ECHO_PIN 8
#define MAX_DISTANCE 75

#define Kp 20//175//175
#define Ki 1//3//3
#define Kd 20//40//175
#define sampleTime  0.00004//0.00004//0.00004
#define targetAngle -4.2//-4.45

MPU6050 mpu;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int16_t accX, accZ, gyroY;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;
volatile byte count = 0;
int distanceCm;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  Serial.println(leftMotorSpeed);
  Serial.println(rightMotorSpeed);
  if (leftMotorSpeed >= 0) {
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
    analogWrite(leftMotorDirPin, 0);
    analogWrite(test2, 255);
  }
  else {
    //analogWrite(leftMotorPWMPin, 255 + leftMotorSpeed);
    analogWrite(leftMotorPWMPin, -1 * leftMotorSpeed);
    analogWrite(leftMotorDirPin, 255);
    analogWrite(test2, 0);
  }
  if (rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    analogWrite(rightMotorDirPin, 0);
    analogWrite(test, 255);
  }
  else {
    //analogWrite(rightMotorPWMPin, 255 + rightMotorSpeed);
    analogWrite(rightMotorPWMPin, -1 * rightMotorSpeed);
    analogWrite(rightMotorDirPin, 255);
    analogWrite(test, 0);
  }
}

void init_PID() {
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  // set compare match register to set sample time 5ms
  OCR1A = 9999;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
  // set the motor control and PWM pins to output mode
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT);
  pinMode(test, OUTPUT);
  pinMode(test2, OUTPUT);
  // set the status LED to output mode
  pinMode(13, OUTPUT);
  // initialize the MPU6050 and set offset values
  mpu.initialize();
  mpu.setXAccelOffset(-4544);
  mpu.setZAccelOffset(1549);
  mpu.setYGyroOffset(-28);
  // initialize PID sampling loop
  init_PID();
  Serial.begin(9600);
}

void loop() {
  // read acceleration and gyroscope values
  accX = mpu.getAccelerationX();
  accZ = mpu.getAccelerationZ();
  gyroY = mpu.getRotationY();
  // set motor power after constraining it
  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower, motorPower);
  // measure distance every 100 milliseconds
  if ((count % 20) == 0) {
    distanceCm = sonar.ping_cm();
  }
  if ((distanceCm < 20) && (distanceCm != 0)) {
    setMotors(-motorPower, motorPower);
  }
  Serial.println(currentAngle);
}
// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accX, accZ) * RAD_TO_DEG;
  gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * sampleTime;
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accAngle);
  //Serial.println(currentAngle);
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp * (error) + Ki * (errorSum) * sampleTime - Kd * (currentAngle - prevAngle) / sampleTime;
  prevAngle = currentAngle;
  // toggle the led on pin13 every second
  count++;
  if (count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}
