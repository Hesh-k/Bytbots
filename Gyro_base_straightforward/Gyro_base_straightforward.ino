#include <Wire.h>

// Motor A connections
const int motor1A = 2;  // Connect this to IN1 on the L9110S module
const int motor1B = 3;  // Connect this to IN2 on the L9110S module

// Motor B connections
const int motor2A = 4;  // Connect this to IN3 on the L9110S module
const int motor2B = 5;  // Connect this to IN4 on the L9110S module

// Motor speed control pins
const int leftSpeed = 9;  // Connect this to the speed control input for the left motor on the L9110S module
const int rightSpeed = 5; // Connect this to the speed control input for the right motor on the L9110S module

const int MPU = 0x68; 
float AccX, AccY, AccZ; 
float GyroX, GyroY, GyroZ; 
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; 
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

const int maxSpeed = 255; 
const int minSpeed = 160; 
float angle;
float targetAngle = 0;
int equilibriumSpeed = 248; 
int leftSpeedVal;
int rightSpeedVal;
bool isDriving = false; 
bool prevIsDriving = true; 
bool paused = false;

// Function declarations
void readAcceleration();
void readGyro();
void stopCar();
void forward();
void left();
void right();
void calculateError();
int changeSpeed(int motorSpeed, int increment);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  calculateError();
  delay(20);

  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  pinMode(right1, OUTPUT);
  pinMode(right2, OUTPUT);

  pinMode(leftSpeed, OUTPUT);
  pinMode(rightSpeed, OUTPUT);

  currentTime = micros();
}

void loop() {
  // ... (unchanged code)

}

// ... (unchanged code)

void readAcceleration() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
}

void readGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
}

// ... (unchanged code)


