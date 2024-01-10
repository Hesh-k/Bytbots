#include <Wire.h>

const int left1 = 3;  // Connect this to the IN1 on the L9110S module
const int left2 = 2;  // Connect this to the IN2 on the L9110S module
const int right1 = 8; // Connect this to the IN3 on the L9110S module
const int right2 = 4; // Connect this to the IN4 on the L9110S module

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
  readAcceleration();
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; 
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;

  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000;
  readGyro();
  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;
  gyroAngleX += GyroX * elapsedTime;
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;

  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  angle = roll;

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'w') {
      Serial.println("forward");
      isDriving = true;
    } else if (c == 'a') {
      Serial.println("left");
      targetAngle += 90;
      if (targetAngle > 180) {
        targetAngle -= 360;
      }
      isDriving = false;
    } else if (c == 'd') {
      Serial.println("right");
      targetAngle -= 90;
      if (targetAngle <= -180) {
        targetAngle += 360;
      }
      isDriving = false;
    } else if (c == 'q') {
      Serial.println("stop");
      isDriving = false;
    } else if (c == 'i') {
      Serial.print("angle: ");
      Serial.println(angle);
      Serial.print("targetAngle: ");
      Serial.println(targetAngle);
      Serial.print("GyroX: ");
      Serial.println(GyroX);
      Serial.print("elapsedTime (in ms): ");
      Serial.println(elapsedTime * pow(10, 3));
      Serial.print("equilibriumSpeed: ");
      Serial.println(equilibriumSpeed);
    } else if (c == 'p') {
      paused = !paused;
      stopCar();
      isDriving = false;
      Serial.println("key p was pressed, which pauses/unpauses the program");
    }
  }

  static int count;
  static int countStraight;
  if (count < 6) {
    count++;
  } else {
    count = 0;
    if (!paused) {
      if (isDriving != prevIsDriving) {
        leftSpeedVal = equilibriumSpeed;
        countStraight = 0;
        Serial.print("mode changed, isDriving: ");
        Serial.println(isDriving);
      }
      if (isDriving) {
        if (abs(targetAngle - angle) < 3) {
          if (countStraight < 20) {
            countStraight++;
          } else {
            countStraight = 0;
            equilibriumSpeed = leftSpeedVal;
            Serial.print("EQUILIBRIUM reached, equilibriumSpeed: ");
            Serial.println(equilibriumSpeed);
          }
        } else {
          countStraight = 0;
        }
        driving();
      } else {
        rotate();
      }
      prevIsDriving = isDriving;
    }
  }

}

void driving() {
  int deltaAngle = round(targetAngle - angle);
  forward();
  if (deltaAngle != 0) {
    controlSpeed();
    rightSpeedVal = maxSpeed;
    analogWrite(rightSpeed, rightSpeedVal);
    analogWrite(leftSpeed, leftSpeedVal);
  }
}

void controlSpeed() {
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;

  if (deltaAngle > 30) {
    targetGyroX = 60;
  } else if (deltaAngle < -30) {
    targetGyroX = -60;
  } else {
    targetGyroX = 2 * deltaAngle;
  }

  if (round(targetGyroX - GyroX) == 0) {
    ;
  } else if (targetGyroX > GyroX) {
    leftSpeedVal = changeSpeed(leftSpeedVal, -1);
  } else {
    leftSpeedVal = changeSpeed(leftSpeedVal, 1);
  }
}

void rotate() {
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;
  if (abs(deltaAngle) <= 1) {
    stopCar();
  } else {
    if (angle > targetAngle) {
      left();
    } else if (angle < targetAngle) {
      right();
    }

    if (abs(deltaAngle) > 30) {
      targetGyroX = 60;
    } else {
      targetGyroX = 2 * abs(deltaAngle);
    }

    if (round(targetGyroX - abs(GyroX)) == 0) {
      ;
    } else if (targetGyroX > abs(GyroX)) {
      leftSpeedVal = changeSpeed(leftSpeedVal, 1);
    } else {
      leftSpeedVal = changeSpeed(leftSpeedVal, -1);
    }
    rightSpeedVal = leftSpeedVal;
    analogWrite(rightSpeed, rightSpeedVal);
    analogWrite(leftSpeed, leftSpeedVal);
  }
}

int changeSpeed(int motorSpeed, int increment) {
  motorSpeed += increment;
  if (motorSpeed > maxSpeed) {
    motorSpeed = maxSpeed;
  } else if (motorSpeed < minSpeed) {
    motorSpeed = minSpeed;
  }
  return motorSpeed;
}

void calculateError() {
  c = 0;
  while (c < 200) {
    readAcceleration();
    AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
    AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
    c++;
  }
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;

  while (c < 200) {
    readGyro();
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
    c++;
  }
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  Serial.println("The gyroscope setting in MPU6050 has been calibrated");
}

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

void stopCar() {
  digitalWrite(right1, LOW);
  digitalWrite(right2, LOW);
  digitalWrite(left1, LOW);
  digitalWrite(left2, LOW);

  analogWrite(rightSpeed, 0);
  analogWrite(leftSpeed, 0);
}

void forward() {
  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);

  analogWrite(rightSpeed, rightSpeedVal);
  analogWrite(leftSpeed, leftSpeedVal);
}

void left() {
  digitalWrite(right1, LOW);
  digitalWrite(right2, HIGH);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);

  analogWrite(rightSpeed, rightSpeedVal);
  analogWrite(leftSpeed, leftSpeedVal);
}

void right() {
  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);
  digitalWrite(left1, LOW);
  digitalWrite(left2, HIGH);

  analogWrite(rightSpeed, rightSpeedVal);
  analogWrite(leftSpeed, leftSpeedVal);
}


