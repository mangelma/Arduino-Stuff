#include <SharpDistSensor.h>
#include <Wire.h>
#include <Romi32U4.h>
#include <LSM6.h>

Romi32U4ButtonA buttonA;
Romi32U4LCD lcd;
Romi32U4Motors motors;
Romi32U4Buzzer buzzer;
Romi32U4Encoders encoders;
LSM6 imu;
#define COLLISION_DIST 4
#define E 2.71828
#define MOUNT_DIST 1.26
#define SIDE_THRESH 101

uint32_t turnAngle = 0;               // rotationresist
int turnAngleInDegrees;               // rotationresist
int16_t turnRate;                     // rotationresist
int16_t gyroOffset;                   // rotationresist
uint16_t gyroLastUpdate = 0;          // rotationresist
int leftSpeed = 100; // initial speeds
int rightSpeed = 120; // initial speeds
int gyroResetSteps = 1024;
int stuckOrNot = 0;
int sideAvoidDegrees = 3;
int beep = 1; // use for demo and video, not for general testing unless want to annoy people
int turningSpeed = 100;
int minConstraint = 50; // for speeds
int maxConstraint = 125; // for speeds
float pFactor = 0.01; // used in backToInitial, how aggressively we are turning

float sorteD(float, int);
float sideDist[4];
float desiredHead [4];
float w = 0, x = 0, y = 0, z = 0;

const byte sensorPinR = A0;
const byte sensorPinM = A2;
const byte sensorPinL = A3;
const byte mediumFilterWindowSize = 5;
SharpDistSensor sensorR(sensorPinR, mediumFilterWindowSize);
SharpDistSensor sensorM(sensorPinM, mediumFilterWindowSize);
SharpDistSensor sensorL(sensorPinL, mediumFilterWindowSize);

void setup() {
  Serial.begin(9600);
  Serial.println(sensorL.getDist());
  Serial.println(sensorR.getDist());
  lcd.print("Calibrating");
  sensorM.setModel(SharpDistSensor::GP2Y0A60SZLF_5V);
  Wire.begin();
  gyroReset();
  turnSensorReset();
}

void loop() {
  updateAll(); // to print stuff on lcd before we get going
  if (buttonA.isPressed())
  {
    lcd.clear();
    delay(700);
    updateAll();
    while(true) {
      getGoing();
      Serial.println(senseLeft());
      Serial.println(senseRight());
      Serial.println(sensorDist());
      activeAvoid(sideDist, 4, desiredHead, 4, &w, &x, &y, &z);
    }
  }
}

// inital avoidance algorithem
void activeAvoid(float sideDist[], int size, float desiredHead[], int, float *w, float *x, float *y, float *z) { 
  updateAll();
  float bestRoute = 0;
  if (tooClose() == 1)
  {
    reversing(2, 50);
    updateAll();
    quickCheck(sideDist, 4, desiredHead, 4, w, x, y, z);
    updateAll();
    bestRoute = sorteD(sideDist, 4);
    updateAll();
    makeMoves(bestRoute, w, x, y, z);
  }
}

// robot turns left and right to check distances of obstacles on each side and stores to an array
void quickCheck(float sideDist[], int isize, float desiredHead[], int dSide, float *w, float *x, float *y, float *z) {
  if (beep == 1) {
      buzzer.playFrequency(800, 100, 10);
    }
  desiredHead[0] = 90; // -90 from start
  desiredHead[1] = 60; // -30 from start
  desiredHead[2] = 60; // +30 from start
  desiredHead[3] = 60; // +90 from start
  stationaryLeftTurn(desiredHead[0], turningSpeed);
  sideDist[0] = sensorDist();
  *w =  sideDist[0];
  updateAll();
  stationaryRightTurn(desiredHead[1], turningSpeed);
  sideDist[1] = sensorDist();
  *w =  sideDist[0];
  updateAll();
  stationaryRightTurn(desiredHead[2], turningSpeed);
  sideDist[2] = sensorDist();
  *y =  sideDist[2];
  updateAll();
  stationaryRightTurn(desiredHead[3], turningSpeed);
  sideDist[3] = sensorDist();
  *z =  sideDist[3];
  updateAll();
}

// robot moves if sensor distance is greater than collision distance and heading is in forward direction
void getGoing() {  

  while (sensorDist() > COLLISION_DIST && (senseLeft() != 1) && (senseRight() != 1))
  {
    updateAll();
    motors.setSpeeds(leftSpeed, rightSpeed);
    updateAll();
       

    // no obstacles, turning towards initial heading
    if ((senseLeft() == 0) && (senseRight() == 0) && (sensorDist() > COLLISION_DIST)) {
      Serial.println("no obstacles");
      backToInitial();
    }
  }

  while (senseRight() == 1) {
    if (beep == 1) {
      buzzer.playFrequency(800, 100, 10);
    }
    Serial.println("turnin left");
    stationaryLeftTurn(sideAvoidDegrees, turningSpeed);
  }

  while (senseLeft() == 1) {
    if (beep == 1) {
      buzzer.playFrequency(800, 100, 10);
    }
    Serial.println("turning right");
    stationaryRightTurn(sideAvoidDegrees, turningSpeed);
  }

}

// adjusting motor speeds based on current heading
void backToInitial() {

  updateAll();
  float speedAdjust = (abs(gyroHeading()) - speedAdjust) * pFactor;

  // slowly turning towards original heading
  if (gyroHeading() > 0) {
    leftSpeed = leftSpeed + speedAdjust;
    rightSpeed = rightSpeed - speedAdjust;
  }

  // slowly turning towards original heading
  if (gyroHeading() < 0) {
    leftSpeed = leftSpeed - speedAdjust;
    rightSpeed = rightSpeed + speedAdjust;
  }

  // slowing down if we are near obstacles
  if (gyroHeading() < 5 && gyroHeading() > -5) {
    leftSpeed = sensorDist() * 25 - 100;
    rightSpeed = sensorDist() * 25 - 100;
  }

  // constraining
  if (leftSpeed < 0) {
    leftSpeed = constrain(leftSpeed, -maxConstraint, -minConstraint);
  } else {
    leftSpeed = constrain(leftSpeed, minConstraint, maxConstraint);
  }

  // constraining
  if (rightSpeed < 0) {
    rightSpeed = constrain(rightSpeed, -maxConstraint, -minConstraint);
  } else {
    rightSpeed = constrain(rightSpeed, minConstraint, maxConstraint);
  }

  motors.setSpeeds(leftSpeed, rightSpeed);
}

//  bubble sort to find the distance giving Romi the most range to move when doing its quickCheck func.
float sorteD(float sideDist[], int size) {
  int i = 0, j = 0;
  float temp = 0, swap = 0;
  for (j = 0; j < 3; j++)
  {
    swap = 0;   //needed to verify that a change happened
    for (i = 0; i < 3; i++)
    {
      if (sideDist[i] > sideDist[i + 1])
      {
        temp =  sideDist[i];
        sideDist[i] = sideDist[i + 1];
        sideDist[i + 1] = temp;
        swap = 1;
      }
    }
    if (swap == 0)  //condition only met if the if statement above is FALSE
    {
      break;
    }
  }
  return sideDist[3];
}

// calculating distance from analog sensor
float sensorDist() {
  unsigned int rangeAway;
  float calcDistance;
  rangeAway = sensorM.getDist();
  calcDistance = 4.2443 * pow (E, 0.0022 * rangeAway) - MOUNT_DIST;  // equation based on our sensor graph
  return calcDistance;
}

int senseLeft() {
  int i = 0;
  if (sensorL.getDist() < SIDE_THRESH) {
    i = 0;
  } else {
    i = 1;
  }
  return i;
}

int senseRight() {
  int i = 0;
  if (sensorR.getDist() < SIDE_THRESH) {
    i = 0;
  } else {
    i = 1;
  }
  return i;
}

void makeMoves(float route, float *w, float *x, float *y, float *z) {
  if (beep == 1) {
      buzzer.playFrequency(800, 100, 10);
    }
  if (stuckOrNot == 2) {
    stationaryLeftTurn(180, turningSpeed);
    stuckOrNot = 0;
  } else {
    if (route == *w)
    {
      stationaryLeftTurn(desiredHead[1] + desiredHead[2] + desiredHead[3], turningSpeed);
      stuckOrNot++;
    }
    else if (route == *x)
    {
      stationaryLeftTurn(desiredHead[3] + desiredHead[2], turningSpeed);
    }
    else if (route == *y)
    {
      stationaryLeftTurn(desiredHead[3], turningSpeed);
    }
    else if (route == *z)
    {
      stuckOrNot++;
    }
  }
}

//checks if we are too close and sends back 0 or 1
int tooClose() {
  int L = 0;
  L = (sensorDist() <= COLLISION_DIST) ? 1 : 0;
  updateAll();
  return L;
}

// will make our updates from gyro and sensor before printing to LCD
void updateAll() {
  gyroHeading();
  printInfo();
}

void stationaryRightTurn(float angle, int speed) {
  float distance = angle * 0.05;
  int countsLeft = 0;
  int countsRight = 0;
  float leftTarget = distance * 161.2;
  float rightTarget = distance * -161.2;
  int countsLeftBefore = encoders.getCountsLeft();
  int countsRightBefore = encoders.getCountsRight();
  int lSpeed = speed;
  while (countsLeft <= leftTarget && countsRight >= rightTarget) {
    
    updateAll();
    motors.setSpeeds(lSpeed, -speed);
    int leftAfter = encoders.getCountsLeft();
    int rightAfter = encoders.getCountsRight();
    countsLeft = leftAfter - countsLeftBefore;
    countsRight = rightAfter - countsRightBefore;
  }
  motors.setSpeeds(0, 0); // stop
}

void stationaryLeftTurn(float angle, int speed) {
  float distance = angle * 0.05;
  int countsLeft = 0;
  int countsRight = 0;
  float leftTarget = distance * -161.2;
  float rightTarget = distance * 161.2;
  int countsLeftBefore = encoders.getCountsLeft();
  int countsRightBefore = encoders.getCountsRight();
  int rSpeed = speed;
  while (countsLeft >= leftTarget && countsRight <= rightTarget) {
    updateAll();
    motors.setSpeeds(-speed, rSpeed);
    int leftAfter = encoders.getCountsLeft();
    int rightAfter = encoders.getCountsRight();
    countsLeft = leftAfter - countsLeftBefore;
    countsRight = rightAfter - countsRightBefore;
  }
  motors.setSpeeds(0, 0); // stop
}

void reversing(float distance, int speed) {
  int countsLeft = 0;
  int countsRight = 0;
  float leftTarget = distance * -161.2;
  float rightTarget = distance * -161.2;
  int countsLeftBefore = encoders.getCountsLeft();
  int countsRightBefore = encoders.getCountsRight();
  while (countsLeft > leftTarget && countsRight > rightTarget) {
    if (beep == 1) {
      buzzer.playFrequency(800, 100, 10);
    }
    updateAll();
    motors.setSpeeds(-speed, -speed);
    int leftAfter = encoders.getCountsLeft();
    int rightAfter = encoders.getCountsRight();
    countsLeft = leftAfter - countsLeftBefore;
    countsRight = rightAfter - countsRightBefore;
  }
  motors.setSpeeds(0, 0); // stop
}

void printInfo() {
  lcd.gotoXY(0, 0);
  lcd.print(sensorDist());
  lcd.print("  ");
  lcd.gotoXY(0, 1);
  lcd.print(gyroHeading());
  lcd.print(" ");
}

int gyroHeading() {
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;
  int32_t d = (int32_t)turnRate * dt;
  turnAngle += (int64_t)d * 7340032 / 17578125;
  turnAngleInDegrees = (((int32_t)turnAngle >> 16) * 360) >> 16;
  return turnAngleInDegrees;
}

void turnSensorReset() { // Modified from RotationResist
  gyroLastUpdate = micros();
  turnAngle = 0;
}

void gyroReset() { // Modified from RotationResist
  imu.init();
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b10001000);
  delay(500);
  int32_t total = 0;
  for (uint16_t i = 0; i < gyroResetSteps; i++)
  {
    while (!imu.readReg(LSM6::STATUS_REG) & 0x08);
    imu.read();
    total += imu.g.z;
  }
  gyroOffset = total / gyroResetSteps;
  turnSensorReset();
  turnAngleInDegrees = (((int32_t)turnAngle >> 16) * 360) >> 16;
}
