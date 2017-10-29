#include <SharpDistSensor.h>
//#include <Wire.h>
#include <Romi32U4.h>
//#include <LSM6.h>
uint32_t turnAngle = 0;               // rotationresist
int turnAngleInDegrees;               // rotationresist
int16_t turnRate;                     // rotationresist
int16_t gyroOffset;                   // rotationresist
uint16_t gyroLastUpdate = 0;          // rotationresist
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Romi32U4LCD lcd;
Romi32U4Motors motors;
Romi32U4Buzzer buzzer;
Romi32U4Encoders encoders;
//LSM6              imu;
int countsLeft = 0;
int countsRight = 0;
int delayBetweenMoves = 100;
unsigned int distanceR;
unsigned int distanceL;
unsigned int distanceM;
const byte sensorPinR = A0;
const byte sensorPinM = A2;
const byte sensorPinL = A3;
const byte mediumFilterWindowSize = 5;
SharpDistSensor sensorR(sensorPinR, mediumFilterWindowSize);
SharpDistSensor sensorM(sensorPinM, mediumFilterWindowSize);
SharpDistSensor sensorL(sensorPinL, mediumFilterWindowSize);


void setup() {
  Serial.begin(9600);
  lcd.print("ready");
  sensorM.setModel(SharpDistSensor::GP2Y0A60SZLF_5V);
  //Wire.begin();
 // gyroReset();
  delay(500);
  //turnSensorReset();
}

void loop() {
updateDistances();
 if (buttonA.isPressed()) {
    lcd.clear();
    updateDistances();
    delay(1000);
    updateDistances();
    exploreTheWorld();
    
  }
}

void updateDistances() {
    distanceR = sensorR.getDist();
    distanceM = sensorM.getDist();
    distanceL = sensorL.getDist();
    Serial.println(distanceR);
    Serial.println(distanceL);
    Serial.println(distanceM);
    Serial.println("R / L / M:");
    //turnSensorUpdate();
}

void exploreTheWorld() {
  int i = 0;
  while (i < 200) {
     lcd.print(i);
     updateDistances();
     motors.setSpeeds(100, 110);

    int angleDiff = 45 - turnAngleInDegrees;
     
     if (distanceR > 100) {
      motors.setSpeeds(-50,-50);
      delay(50);
      stationaryLeftTurn(10, 75);
     }

      else if (distanceL > 100) {
       motors.setSpeeds(-50,-50);
       delay(50);
       stationaryRightTurn(20, 75);
     }

      else if (distanceM < 100) {
        buzzer.playFrequency(800, 100, 10); 
        motors.setSpeeds(-50,-50);
        delay(100);
        stationaryRightTurn(45, 75);
    
     }

     else { 
      // going straight for some time
       delay(50);
       i++;
       }

     
     lcd.clear();
  }
  
  motors.setSpeeds(0,0);
    
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
  //Serial.println(lSpeed);
  while (countsLeft <= leftTarget && countsRight >= rightTarget) {
    // turnSensorUpdate();
    motors.setSpeeds(lSpeed, -speed);
    int leftAfter = encoders.getCountsLeft();
    int rightAfter = encoders.getCountsRight();
    countsLeft = leftAfter - countsLeftBefore;
    countsRight = rightAfter - countsRightBefore;
    // adjustment of left motor speed
    if (countsLeft < abs(countsRight) ) { 
      lSpeed = speedUpAdjust(countsLeft, countsRight, lSpeed);
      }  
    if (countsLeft > abs(countsRight) ) {
      lSpeed = slowDownAdjust(countsLeft, countsRight, lSpeed);
      }  
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
    // turnSensorUpdate();
    motors.setSpeeds(-speed, rSpeed);
    int leftAfter = encoders.getCountsLeft();
    int rightAfter = encoders.getCountsRight();
    countsLeft = leftAfter - countsLeftBefore; 
    countsRight = rightAfter - countsRightBefore;
    // adjustment of right motor speed
    if ( abs(countsLeft) > countsRight) { 
      rSpeed = speedUpAdjust(countsLeft, countsRight, rSpeed);
      }  
    if ( abs(countsLeft) < countsRight) {
      rSpeed = slowDownAdjust(countsLeft, countsRight, rSpeed);
      }
  }
  motors.setSpeeds(0, 0); // stop
}

float Kp = 25;
float Ki = 0;

float speedUpAdjust(int countsLeft, int countsRight, float speed) {
  int e = abs(countsLeft) - abs(countsRight);
  speed = abs(e) * Kp + speed * Ki;
  speed = constrain(speed, 25, 75);
  return speed;
}

float slowDownAdjust(int countsLeft, int countsRight, float speed) {
  int e = abs(countsRight) - abs(countsLeft);
  speed = abs(e) / Kp - speed * Ki;
  speed = constrain(speed, 25, 75);
  return speed;
}

void goStraight(int speed) {

  int countsLeft = 0;
  int countsRight = 0;
  int countsLeftBefore = encoders.getCountsLeft();
  int countsRightBefore = encoders.getCountsRight();
  int rSpeed = speed;
    motors.setSpeeds(speed, rSpeed);
    int leftAfter = encoders.getCountsLeft();
    int rightAfter = encoders.getCountsRight();
    countsLeft = leftAfter - countsLeftBefore; 
    countsRight = rightAfter - countsRightBefore;
    // adjustment of right motor speed
    if ( abs(countsLeft) > countsRight) { 
      rSpeed = speedUpAdjust(countsLeft, countsRight, rSpeed);
      }  
    if ( abs(countsLeft) < countsRight) {
      rSpeed = slowDownAdjust(countsLeft, countsRight, rSpeed);
      }
}

/*
void turnSensorUpdate() { // Modified from RotationResist
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;
  int32_t d = (int32_t)turnRate * dt;
  turnAngle += (int64_t)d * 7340032 / 17578125;
  turnAngleInDegrees = (((int32_t)turnAngle >> 16) * 360) >> 16;
  lcd.gotoXY(0, 0);
  lcd.print(turnAngleInDegrees);
  lcd.print(" ");
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
    while(!imu.readReg(LSM6::STATUS_REG) & 0x08);
    imu.read();
    total += imu.g.z;
    Serial.println(i % 10);    
  }
  gyroOffset = total / 1024;
  turnSensorReset();
  turnAngleInDegrees = (((int32_t)turnAngle >> 16) * 360) >> 16;
  buzzer.playFrequency(800, 100, 10); // beep to indicate calibration complete
}
*/
