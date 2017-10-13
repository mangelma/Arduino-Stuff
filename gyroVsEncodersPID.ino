#include <Wire.h>
#include <Romi32U4.h>
#include <LSM6.h>

uint32_t turnAngle = 0;               // rotationresist
int turnAngleInDegrees;               // rotationresist
int16_t turnRate;                     // rotationresist
int16_t gyroOffset;                   // rotationresist
uint16_t gyroLastUpdate = 0;          // rotationresist
float degreesToInchesCW = 0.05088;    // this is also adjusted actively, but initial guess
float degreesToInchesCCW = 0.05113;   // this is also adjusted actively, but initial guess
float Kp = 25;
float Ki = 0;

Romi32U4Encoders  encoders;
Romi32U4LCD       lcd;
Romi32U4ButtonA   buttonA;
Romi32U4ButtonB   buttonB;
Romi32U4ButtonC   buttonC;
Romi32U4Buzzer    buzzer;
Romi32U4Motors    motors;
LSM6              imu;

void setup() {
  lcd.clear();
  Serial.begin(9600);
  Wire.begin();
  gyroReset();
  delay(500);
  turnSensorReset();
  buzzer.playFrequency(800, 100, 10); // beep to indicate calibration done
  randomSeed(analogRead(4));
}

void loop() {
  turnSensorUpdate(); // update as much as possible

 if (buttonA.isPressed()) { // turning from angle to starting orientation
    delay(500);             // not going "immediately" because that would cause some error with fingers etc
    turnSensorUpdate();
    int askedAngle = turnAngleInDegrees; // save the asked angle before start to rotate
    
    if (turnAngleInDegrees > 0) {
      stationaryRightTurn(turnAngleInDegrees, 50);
      Serial.println(turnAngleInDegrees);
      turnSensorUpdate();
    }
    
    if (turnAngleInDegrees < 0) {       
      stationaryLeftTurn(abs(turnAngleInDegrees), 50);
      turnSensorUpdate();
    }  
    
    lcd.gotoXY(0,1);
    lcd.print("aa: ");
    lcd.print(askedAngle);
    lcd.print(" ");  
  }

  if (buttonB.isPressed()) { calibDtoI(); }

  if (buttonC.isPressed()) { lcd.clear(); gyroReset(); }
     
  } // end of loop()

void calibDtoI() {
  int e = 1;
  gyroReset();
  turnSensorUpdate();
  int randDegrees = random(90, 175); // degree from 90 to 180
  lcd.gotoXY(5, 0);
  lcd.print(randDegrees);
  lcd.print(" ");
  stationaryRightTurn(randDegrees, 50);                 // turn CW with speed of 50
  turnSensorUpdate();
  e = turnAngleInDegrees + randDegrees;                 // how much difference between set and measured
  degreesToInchesCW = degreesToInchesCW + 0.00005 * e;  // adjust the dToI factor
  lcd.gotoXY(0, 1);
  lcd.print(String(degreesToInchesCW, 6));
  delay(500);
  turnSensorUpdate();                                   // this should be called as much as possible..
  stationaryLeftTurn(randDegrees, 50);                  // turn CCW with speed 50
  turnSensorUpdate();
  // we should be back to zero, if not, adjust
  degreesToInchesCCW = degreesToInchesCCW - 0.00005 * turnAngleInDegrees; 
  lcd.gotoXY(0, 1);
  lcd.print(String(degreesToInchesCCW, 6));
  lcd.gotoXY(0, 0);
  lcd.print(String(degreesToInchesCW, 5));
  buzzer.playFrequency(400, 100, 10); // beep to indicate calibration complete
}

void stationaryRightTurn(float angle, int speed) {
  float distance = angle * degreesToInchesCW;
  int countsLeft = 0;
  int countsRight = 0;
  float leftTarget = distance * 161.2;
  float rightTarget = distance * -161.2;
  int countsLeftBefore = encoders.getCountsLeft();
  int countsRightBefore = encoders.getCountsRight();
  int lSpeed = speed;
  //Serial.println(lSpeed);
  while (countsLeft <= leftTarget && countsRight >= rightTarget) {
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
    turnSensorUpdate();
  }
   motors.setSpeeds(0, 0); // stop
}

void stationaryLeftTurn(float angle, int speed) {
  float distance = angle * degreesToInchesCCW;
  int countsLeft = 0;
  int countsRight = 0;
  float leftTarget = distance * -161.2;
  float rightTarget = distance * 161.2; 
  int countsLeftBefore = encoders.getCountsLeft();
  int countsRightBefore = encoders.getCountsRight();
  int rSpeed = speed;
  while (countsLeft >= leftTarget && countsRight <= rightTarget) {
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
    turnSensorUpdate();
  }
  motors.setSpeeds(0, 0); // stop
}

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
