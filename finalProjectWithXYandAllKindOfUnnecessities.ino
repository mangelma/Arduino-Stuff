#include <SharpDistSensor.h>
#include <Wire.h>
#include <Romi32U4.h>
#include <LSM6.h>

int beep = 1; // use for demo and video, not for general testing unless want to annoy people
int sideAvoidDegrees = 10; // adjust if tight spaces in demo, otherwise bigger better

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
int turnAngleInDegrees = 0;            // rotationresist
float turnAngleInRadians = 0;
int16_t turnRate;                     // rotationresist
int16_t gyroOffset;                   // rotationresist
uint16_t gyroLastUpdate = 0;          // rotationresist
int leftSpeed = 50; // initial speeds
int rightSpeed = 60; // initial speeds
int gyroResetSteps = 1024;
int stuckOrNot = 0;
int leftTriggered = 0;
int rightTriggered = 0;

int reversingDist = 2; // how many inches to reverse when middle sensor triggers
int turningSpeed = 100;
int minConstraint = 50; // for speeds, did 50/125 in the active ranging demo
int maxConstraint = 125; // for speeds, did 50/125 in the active ranging demo
float pFactor = 0.01; // used in backToInitial, how aggressively we are turning
int turnFlag = 0; // raising makes encoder counting to be ignored

float sorteD(float, int);
float sideDist[4];
float desiredHead [4];
float w = 0, x = 0, y = 0, z = 0;

unsigned long lastSideTrig;
unsigned long previousSideAvoid;

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
  Serial.println(sensorL.getDist());
  Serial.println(sensorR.getDist());
  lcd.clear();
}

void loop() {
  updateAll(); // to print stuff on lcd before we get going
  if (buttonA.isPressed())
  {
    lcd.clear();
    delay(700);
    updateAll();
    while (true) {
      getGoing();
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
    beeping();
    rightTriggered = 0;
    leftTriggered = 0;

    // to get us out of trouble
    if (stuckOrNot == 3) {
      int randNumber = random(90, 270);
      stationaryLeftTurn(randNumber, turningSpeed);
      stuckOrNot = 0;
    } else {
      reversing(reversingDist, 50);
    }
    updateAll();
    beeping();
    quickCheck(sideDist, 4, desiredHead, 4, w, x, y, z);
    updateAll();
    beeping();
    bestRoute = sorteD(sideDist, 4);
    updateAll();
    beeping();
    makeMoves(bestRoute, w, x, y, z);
  }
}

// robot turns left and right to check distances of obstacles on each side and stores to an array
void quickCheck(float sideDist[], int isize, float desiredHead[], int dSide, float *w, float *x, float *y, float *z) {
  beeping();
  desiredHead[0] = 90; // -90 from start
  desiredHead[1] = 60; // -30 from start
  desiredHead[2] = 60; // +30 from start
  desiredHead[3] = 60; // +90 from start
  beeping();
  stationaryLeftTurn(desiredHead[0], turningSpeed);
  sideDist[0] = sensorDist();
  *w =  sideDist[0];
  updateAll();
  beeping();
  stationaryRightTurn(desiredHead[1], turningSpeed);
  sideDist[1] = sensorDist();
  *w =  sideDist[0];
  updateAll();
  beeping();
  stationaryRightTurn(desiredHead[2], turningSpeed);
  sideDist[2] = sensorDist();
  *y =  sideDist[2];
  updateAll();
  beeping();
  stationaryRightTurn(desiredHead[3], turningSpeed);
  sideDist[3] = sensorDist();
  *z =  sideDist[3];
  updateAll();
}

void beeping() {
  if (beep == 1) {
    buzzer.playFrequency(440, 50, 15);
  }
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
      backToInitial();
    }
  }

  while (senseRight() == 1) {
    lastSideTrig = millis();
    beeping();
    stationaryLeftTurn(sideAvoidDegrees, turningSpeed);

    // avoid wall following back to start
    if (abs(turnAngleInDegrees) > 90) {
      rightTriggered++;
    }
  }

  while (senseLeft() == 1) {
    lastSideTrig = millis();
    beeping();
    stationaryRightTurn(sideAvoidDegrees, turningSpeed);

    // avoid wall following back to start
    if (abs(turnAngleInDegrees) > 90) {
      leftTriggered++;
    }

  }

  // this should be enough to get out of local minimum, while keeping us not going back to start
  if (leftTriggered >= 15 || rightTriggered >= 15) {
    stationaryLeftTurn(180, turningSpeed);
    leftTriggered = 0;
    rightTriggered = 0;
  }

  Serial.println();
  Serial.println(previousSideAvoid);
  Serial.println(lastSideTrig);
  Serial.println(lastSideTrig - previousSideAvoid);
  
  if ((lastSideTrig - previousSideAvoid) <= 500) {
    sideAvoidDegrees = sideAvoidDegrees / 2;
  } else {
    buzzer.playFrequency(400, 50, 15);
    sideAvoidDegrees = 10;
  }

  // store last time
  previousSideAvoid = lastSideTrig;


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

  //Serial.println();
  //Serial.println(speedAdjust);
  //Serial.println(leftSpeed);
  //Serial.println(rightSpeed);
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
  beeping();
  if (route == *w)
  {
    stationaryLeftTurn(desiredHead[1] + desiredHead[2] + desiredHead[3], turningSpeed);
    stuckOrNot++;
  }
  else if (route == *x)
  {
    stationaryLeftTurn(desiredHead[3] + desiredHead[2], turningSpeed);
    stuckOrNot++;
  }
  else if (route == *y)
  {
    stationaryLeftTurn(desiredHead[3], turningSpeed);
    stuckOrNot++;
  }
  else if (route == *z)
  {
    stuckOrNot++;
  }
}

//checks if we are too close and sends back 0 or 1
int tooClose() {
  int L = 0;
  L = (sensorDist() <= COLLISION_DIST) ? 1 : 0;
  updateAll();
  return L;
}

int lCounts = 0;
int rCounts = 0;
int lCountsBefore = encoders.getCountsLeft();
int rCountsBefore = encoders.getCountsRight();
unsigned long lCountsPrevious = 0;
unsigned long rCountsPrevious = 0;
float lDistanceTravelled = 0;
float rDistanceTravelled = 0;
float xVector = 0;
float yVector = 0;
float xCoord = 0;
float yCoord = 0;
unsigned long previousMillis = 0;

// will make our updates from gyro and sensor before printing to LCD, encoder values
void updateAll() {
  gyroHeading();
  unsigned long lCountsCurrent = encoders.getCountsLeft();
  unsigned long rCountsCurrent = encoders.getCountsRight();
  unsigned long leftTicks = lCountsCurrent - lCountsPrevious;
  unsigned long rightTicks = rCountsCurrent - rCountsPrevious;

  // less than 50 because turning caused numbers like 4294967290
  // this way we just ignore those without thinking too much, not wanting to overachieve
  if ( (turnFlag == 0) && (leftTicks < 50) && (rightTicks < 50)) {
    lCounts = lCounts + leftTicks;
    rCounts = rCounts + rightTicks;
    lDistanceTravelled = lCounts / 161.2;
    rDistanceTravelled = rCounts / 161.2;
    //float avgDistanceTravelled = (lDistanceTravelled + rDistanceTravelled) / 2;
    xVector = ((leftTicks + rightTicks) / 2) / 161.2 * cos(turnAngleInRadians);
    yVector = ((leftTicks + rightTicks) / 2) / 161.2 * sin(turnAngleInRadians);
    xCoord = xCoord + xVector;
    yCoord = yCoord + yVector;
    //Serial.println();
    //Serial.println(xCoord);
    //Serial.println(yCoord);
    //Serial.println(turnAngleInRadians);
  }

  lCountsPrevious = lCountsCurrent;
  rCountsPrevious = rCountsCurrent;
  float XYdistance = pow(pow(xCoord, 2) + pow(yCoord, 2), 0.5);
  unsigned long currentMillis = millis();

  // Arduino.cc blink without delay -timer
  if (currentMillis - previousMillis >= 100) {
    previousMillis = currentMillis;
    lcd.gotoXY(0, 0);
    //lcd.print(sensorDist(), 1);
    lcd.print("d: ");
    lcd.print("  ");
    lcd.gotoXY(3, 0);
    lcd.print(XYdistance, 0); // clockwise is negative values
    lcd.print(" ");

    lcd.gotoXY(0, 1);
    lcd.print("h:");
    lcd.print(" ");

    lcd.gotoXY(3, 1);
    lcd.print(abs(turnAngleInDegrees));
    lcd.print("  ");
  }



}

void stationaryRightTurn(float angle, int speed) {
  turnFlag = 1;
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
  turnFlag = 0;
}

void stationaryLeftTurn(float angle, int speed) {
  turnFlag = 1;
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
  turnFlag = 0;
}

void reversing(float distance, int speed) {
  turnFlag = 1;
  int countsLeft = 0;
  int countsRight = 0;
  float leftTarget = distance * -161.2;
  float rightTarget = distance * -161.2;
  int countsLeftBefore = encoders.getCountsLeft();
  int countsRightBefore = encoders.getCountsRight();
  while (countsLeft > leftTarget && countsRight > rightTarget) {
    beeping();
    updateAll();
    motors.setSpeeds(-speed, -speed);
    int leftAfter = encoders.getCountsLeft();
    int rightAfter = encoders.getCountsRight();
    countsLeft = leftAfter - countsLeftBefore;
    countsRight = rightAfter - countsRightBefore;
  }
  motors.setSpeeds(0, 0); // stop
  turnFlag = 0;
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
  turnAngleInRadians = turnAngleInDegrees * 0.01745329252;
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
  imu.writeReg(LSM6::CTRL1_XL, 0b10000100);
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
