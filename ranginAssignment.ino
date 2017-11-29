#include <SharpDistSensor.h>
#include <Wire.h>
#include <Romi32U4.h>
#include <LSM6.h>

Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Romi32U4LCD lcd;
Romi32U4Motors motors;
Romi32U4Buzzer buzzer;
Romi32U4Encoders encoders;
LSM6 imu;
#define COLLISION_DIST 5
#define E 2.71828
#define MOUNT_DIST 1.26
#define OFF_TRACK 90
#define SIDE_THRESH 101
#define MAX_HEAD 85

uint32_t turnAngle = 0;               // rotationresist
int turnAngleInDegrees;               // rotationresist
int16_t turnRate;                     // rotationresist
int16_t gyroOffset;                   // rotationresist
uint16_t gyroLastUpdate = 0;          // rotationresist
int countsLeft = 0;
int countsRight = 0;
int delayBetweenMoves = 100;
int i = 0;
int leftSpeed = 200;
int rightSpeed = 220;

float sorteD(float, int);
float sideDist[4];
float desiredHead [4];
float w = 0, x = 0, y = 0,  z = 0;
float Kp = 25, Ki = 0;

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
  lcd.gotoXY(0, 0);
  lcd.print("ready");
  lcd.print("   ");
  delay(1000);
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("setupped");
}

void loop() {
  updateAll();
   if (buttonA.isPressed())
   {
  lcd.clear();
  delay(700);
  for (i = 0; i < 500; i++)
  {
    //Serial.println(senseLeft());
    updateAll();
    getGoing();
    updateAll();
    activeAvoid(sideDist, 4, desiredHead, 4, &w, &x, &y, &z);
    updateAll();
     }
  }
}


void activeAvoid(float sideDist[], int size, float desiredHead[], int, float *w, float *x, float *y, float *z) { //this is the inital avoidance algorithem
  float bestRoute = 0;
  if (tooClose() == 1)
  {
    // getBack()
    buzzer.playFrequency(800, 100, 10);
    motors.setSpeeds(-50, -50);
    delay(500);
    
    updateAll();
    quickCheck(sideDist, 4, desiredHead, 4, w, x, y, z);
    updateAll();
    bestRoute = sorteD(sideDist, 4);
    updateAll();
    makeMoves(bestRoute, w, x, y, z);
  }
}

void quickCheck(float sideDist[], int isize, float desiredHead[], int dSide, float *w, float *x, float *y, float *z) { //robot turns left and right to check distances of obstacles on each side and stores to an array

  //desiredHead[0] = MAX_HEAD - abs(gyroHeading());

  desiredHead[0] = 90;

  stationaryLeftTurn(desiredHead[0], 100);
  farLeft(sideDist, 4);
  *w =  sideDist[0];
  delay(100);
  updateAll();

  //desiredHead[1] = desiredHead[0] / 2;

  desiredHead[1] = 60;
  stationaryRightTurn(desiredHead[1], 100);
  nearLeft(sideDist, 4);
  *w =  sideDist[0];
  delay(100);
  updateAll();

  //desiredHead[2] = abs(gyroHeading()) * 2;
  desiredHead[2] = 60;

  stationaryRightTurn(desiredHead[2], 100);
  farRight(sideDist, 4);
  *y =  sideDist[2];
  delay (100);
  updateAll();

  //desiredHead[3] = desiredHead[2] / 4;
  desiredHead[3] = 60;

  stationaryRightTurn(desiredHead[3], 100);
  nearRight(sideDist, 4);
  *z =  sideDist[3];
  delay (100);
  updateAll();
}

void getGoing() {  //robot moves if sensor distance is greater than collision distance and heading is in forward direction
  //while ((sensorDist() > COLLISION_DIST) && (senseLeft() != 1) && (senseRight() != 1))        // try to incorporare this into the expression((gyroHeading() < 90) || (gyroHeading() > -90)))

  while (sensorDist() > COLLISION_DIST && (senseLeft() != 1) && (senseRight() != 1))
  {
    updateAll();
    motors.setSpeeds(leftSpeed, rightSpeed);
    updateAll();
    //checkSides();

    // no obstacles
    if ((senseLeft() == 0) && (senseRight() == 0) && (sensorDist() > COLLISION_DIST)) {
      backToInitial();
    }

  } // while > collision distance

 
    buzzer.playFrequency(800, 100, 10);
   // Serial.println(sensorDist());

    while (senseRight() == 1) {
      stationaryLeftTurn(1, 50);
    }
    
    while (senseLeft() == 1) {
      stationaryRightTurn(1, 50);
    }
}

void backToInitial() {


  float speedAdjust = (abs(gyroHeading()) - speedAdjust) * 0.01;

  //Serial.println(speedAdjust);
  lcd.gotoXY(0, 1);
  lcd.print(gyroHeading());
  lcd.print(" ");

  // slowly turning towards original heading
  if (gyroHeading() > 0) {
    leftSpeed = leftSpeed + speedAdjust;
    rightSpeed = rightSpeed - speedAdjust;
  }

  if (gyroHeading() < 0) {
    leftSpeed = leftSpeed - speedAdjust;
    rightSpeed = rightSpeed + speedAdjust;
  }

  if (gyroHeading() < 5 && gyroHeading() > -5) {
    leftSpeed = 100;
    rightSpeed = 110;
  }

  //  Serial.println();
  //  Serial.println(leftSpeed);
  //  Serial.println(rightSpeed);
  //  Serial.println(speedAdjust);
  if (leftSpeed < 0) {
    leftSpeed = constrain(leftSpeed, -200, -25);
  } else {
    leftSpeed = constrain(leftSpeed, 25, 200);
  }

  if (rightSpeed < 0) {
    rightSpeed = constrain(rightSpeed, -200, -25);
  } else {
    rightSpeed = constrain(rightSpeed, 25, 200);
  }

  motors.setSpeeds(leftSpeed, rightSpeed);

}

float sorteD(float sideDist[], int size) { //this bubble sort is used to find the distance giving Romi the most range to move when doing its quickCheck func.
  int i = 0, j = 0;

 Serial.println("Unsorted:");
  Serial.println(sideDist[0]);
  Serial.println(sideDist[1]);
  Serial.println(sideDist[2]);
  Serial.println(sideDist[3]);
  
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
  Serial.println("Sorted:");
  Serial.println(sideDist[0]);
  Serial.println(sideDist[1]);
  Serial.println(sideDist[2]);
  Serial.println(sideDist[3]);
  
  return sideDist[3];
}

void farLeft(float sideDist[], int size) {
  sideDist[0] = sensorDist();
  // sideDist[0] = sensorDist() + senseLeft() + senseRight();
}

void nearLeft(float sideDist[], int size) { //writes left side distances to array
  sideDist[1] = sensorDist(); //+ senseLeft() + senseRight();
}

void nearRight(float sideDist[], int size) { //writes right side distances to array
  sideDist[2] = sensorDist(); // + senseLeft() + senseRight();
}

void farRight(float sideDist[], int size) { //writes right side distances to array
  sideDist[3] = sensorDist(); // + senseLeft() + senseRight();
}

float sensorDist() { //active ranging using analog sensor and calculated formula on exponential curve

  unsigned int rangeAway;
  float calcDistance;

  rangeAway = sensorM.getDist();
  calcDistance = 4.2443 * pow (E, 0.0022 * rangeAway) - MOUNT_DIST;  //formula based on our sensor graph

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

  Serial.println("route:");
  Serial.println(route);

  if (route == *w)
  {
    stationaryLeftTurn(desiredHead[1] + desiredHead[2] + desiredHead[3], 75);
  }
  else if (route == *x)
  {
    stationaryLeftTurn(desiredHead[3] + desiredHead[2], 75);
  }
  else if (route == *y)
  {
    stationaryLeftTurn(desiredHead[3], 75);
  }
  else if (route == *z)
  {
    //stationaryLeftTurn(desiredHead[3], 75);
  }
}

int tooClose() {  //checks if we are too close and sends back 0 or 1
  int L = 0;
  L = (sensorDist() <= COLLISION_DIST) ? 1 : 0;
  updateAll();
  return L;
}

// MIGHT NOT BE USED
/*

  void adjustHeading() { //commands avoidance turn according to heading
  int i = 0;

  do  {
    if (gyroHeading() >= 0)
    {
      stationaryLeftTurn(45, 75);
      delay(500);
      updateAll();
      i++;
    }
    if (gyroHeading() < 0)
    {
      stationaryRightTurn(45, 75);
      delay(500);
      updateAll();
      i++;
    }
  } while ((tooClose() == 1) && (i != 3));

  if (i == 3)
  {
    aboutFace();
  }
  }

void aboutFace()  { //turns robot 180 degrees
  stationaryLeftTurn(180, 75);
}

void slightAdjust() {  //gets romi in reverse for a small period to avoid side obstacle
  buzzer.playFrequency(800, 100, 10);
  motors.setSpeeds(-50, -50);
  delay(300);
}

void checkSides() { //checks the presense of an obstacle on the left or right using side sensors
  Serial.println("checking sides L/R:");
  Serial.println(senseLeft());
  Serial.println(senseRight());

  if (senseLeft() == 1)
  {
    //slightAdjust();
    buzzer.playFrequency(800, 100, 10);
    leftSpeed = leftSpeed + 5;
    rightSpeed = -rightSpeed;
  }

  if (senseRight() == 1)
  {
    //slightAdjust();
    buzzer.playFrequency(800, 100, 10);
    rightSpeed = rightSpeed + 5;
    leftSpeed = -leftSpeed;
  }

  motors.setSpeeds(leftSpeed, rightSpeed);

}

*/

// MISC FUNCTIONS
void updateAll() { // will make our updates from gyro and sensor before printing to LCD
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

void printInfo() { //prints information to screen
  lcd.gotoXY(0, 0);
  lcd.print(sensorDist());
  lcd.print("  ");
  lcd.gotoXY(0, 1);
  lcd.print(gyroHeading());
  lcd.print(" ");

  /*
    Serial.println("Debug: left, right, middle, left, right");
    Serial.println(senseLeft());
    Serial.println(senseRight());
    Serial.println(sensorDist());
    Serial.println(sensorL.getDist());
    Serial.println(sensorR.getDist());

  */
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
  for (uint16_t i = 0; i < 1024; i++)
  {
    while (!imu.readReg(LSM6::STATUS_REG) & 0x08);
    imu.read();
    total += imu.g.z;
    //Serial.println(i % 10);
  }
  gyroOffset = total / 1024;
  turnSensorReset();
  turnAngleInDegrees = (((int32_t)turnAngle >> 16) * 360) >> 16;
  buzzer.playFrequency(800, 100, 10); // beep to indicate calibration complete
}
