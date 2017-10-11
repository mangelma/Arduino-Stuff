#include <Wire.h>
#include <Romi32U4.h>
#include <LSM6.h>
uint32_t turnAngle = 0;
int turnAngleInDegrees;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;
const int16_t maxSpeed = 300;
int countsLeft = 0;
int countsRight = 0;
float lCorrection = 13;         
//float pFactor = 0.00000001; // stable   
float pFactor = 0.00001;     
float rCorrection = -5;        
float degreesToInchesCW = 0.05082; 
float degreesToInchesCCW = 0.05098;
Romi32U4Encoders encoders;
Romi32U4LCD lcd;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Romi32U4Buzzer buzzer;
Romi32U4Motors motors;
LSM6 imu;

void setup()
{
  Serial.begin(9600);
  turnSensorSetup();
  delay(500);
  turnSensorReset();
  buzzer.playFrequency(800, 100, 10);
}

void loop()
{
  turnSensorUpdate();

 if (buttonA.isPressed()) { // turning from angle to starting orientation
  
    delay(1000); 
    turnSensorUpdate();
    int askedAngle = turnAngleInDegrees;

    if (turnAngleInDegrees > 0) {
      stationaryRightTurn(turnAngleInDegrees, 50);
    }

    if (turnAngleInDegrees < 0) {       
      stationaryLeftTurn((-1)*turnAngleInDegrees, 50);
    }  

    lcd.gotoXY(0,1);
    lcd.print("o: ");
    lcd.print(askedAngle);
    lcd.print(" ");
  }


  if (buttonB.isPressed()) {  gyroReset(); lcd.clear(); }


  if (buttonC.isPressed()) { // calibration routine
      for (int i=0;i<1;i++) {
        calibDtoI();
        turnSensorUpdate();
        delay(1000);
      }
            
    } // end of buttonC
     
  } // end of loop()


void calibDtoI() {
  
      gyroReset();
      turnSensorUpdate();
      stationaryRightTurn(720, 50);
      turnSensorUpdate();
      degreesToInchesCW = degreesToInchesCW + 0.00003 * turnAngleInDegrees;
      
      lcd.gotoXY(0, 1);
      String dToI =  String(degreesToInchesCW, 5); 
      lcd.print(dToI);

      gyroReset();
      turnSensorUpdate();
      stationaryLeftTurn(720, 50);
      turnSensorUpdate();
      degreesToInchesCCW = degreesToInchesCCW - 0.00003 * turnAngleInDegrees;

      lcd.gotoXY(0, 1);
      dToI =  String(degreesToInchesCCW, 5); 
      lcd.print(dToI);
}

void stopMotors() {
  motors.setSpeeds(0, 0);
}

void stationaryRightTurn(float angle, int speed) {
  float distance = angle * degreesToInchesCW;
  countsLeft = 0;
  countsRight = 0;
  float leftTarget = distance * 161.2;
  float rightTarget = distance * -161.2;
  int countsLeftBefore = encoders.getCountsLeft();
  int countsRightBefore = encoders.getCountsRight();
  int lSpeed = speed;
  Serial.println(lSpeed);
  while (countsLeft <= leftTarget && countsRight >= rightTarget) {
    motors.setSpeeds(lSpeed, -speed);
    int leftAfter = encoders.getCountsLeft();
    int rightAfter = encoders.getCountsRight();
    countsLeft = leftAfter - countsLeftBefore;
    countsRight = rightAfter - countsRightBefore;
    if (countsLeft < abs(countsRight) ) { 
      lSpeed = speedUpAdjust(countsLeft, countsRight, lSpeed);
      }  
    if (countsLeft > abs(countsRight) ) {
      lSpeed = slowDownAdjust(countsLeft, countsRight, lSpeed);
      }  
    turnSensorUpdate();
  }
  Serial.println("R: ");
  Serial.println(countsLeft);
  Serial.println(countsRight);
  lcd.gotoXY(4,0);
  lcd.print(abs(countsLeft)-abs(countsRight));
  stopMotors();
}

void stationaryLeftTurn(float angle, int speed) {
  float distance = angle * degreesToInchesCCW;
  countsLeft = 0;
  countsRight = 0;
  float leftTarget = distance * -161.2;
  float rightTarget = distance * 161.2; 
  int countsLeftBefore = encoders.getCountsLeft();
  int countsRightBefore = encoders.getCountsRight();
  int rSpeed = speed;
  while (countsLeft >= leftTarget && countsRight <= rightTarget) {
    motors.setSpeeds(-speed, rSpeed);
    int leftAfter = encoders.getCountsLeft();
    int rightAfter = encoders.getCountsRight();
    countsLeft = leftAfter - countsLeftBefore; // goes below zero
    countsRight = rightAfter - countsRightBefore; // should stay under
     if ( abs(countsLeft) > countsRight) { 
      rSpeed = speedUpAdjust(countsLeft, countsRight, rSpeed);
      }  
    if ( abs(countsLeft) < countsRight) {
      rSpeed = slowDownAdjust(countsLeft, countsRight, rSpeed);
      }
    turnSensorUpdate();
  }
 Serial.println("L: ");
  Serial.println(countsLeft);
  Serial.println(countsRight);
  lcd.gotoXY(4,0);
  lcd.print(abs(countsLeft)-abs(countsRight));
 stopMotors();
}

float Kp = 25;
float Ki = 0;

float speedUpAdjust(int countsLeft, int countsRight, float speed) {
  Serial.println("speeding, e: ");
  int e = abs(countsLeft) - abs(countsRight);
  Serial.println(e);
  speed = abs(e) * Kp + speed * Ki;
  speed = constrain(speed, 25, 75);
  Serial.println(speed);
  return speed;
}

float slowDownAdjust(int countsLeft, int countsRight, float speed) {
  Serial.println("slowing, e: ");
  int e = abs(countsRight) - abs(countsLeft);
  Serial.println(e);
  speed = abs(e) / Kp - speed * Ki;
  speed = constrain(speed, 25, 75);
  Serial.println(speed);
  return speed;
}

void turnSensorUpdate()
{
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;
  //Serial.println(turnRate);
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;
  int32_t d = (int32_t)turnRate * dt;
  turnAngle += (int64_t)d * 7340032 / 17578125;
  //Serial.println(turnAngle);
  turnAngleInDegrees = (((int32_t)turnAngle >> 16) * 360) >> 16;
  lcd.gotoXY(0, 0);
  lcd.print(turnAngleInDegrees);
  lcd.print(" ");
}

void turnSensorReset() // FROM RESIST ROTATION EXAMPLE
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

void turnSensorSetup() // FROM RESIST ROTATION EXAMPLE
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b10001000);
  lcd.clear();
  lcd.print(F("Gyro cal"));
  delay(500);
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    while(!imu.readReg(LSM6::STATUS_REG) & 0x08);
    imu.read();
    total += imu.g.z;
  }
  gyroOffset = total / 1024;
  turnSensorReset();
  lcd.clear();
}


void gyroReset() {
  //lcd.gotoXY(0,0);
  //lcd.print("rst");
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
  }
  gyroOffset = total / 1024;
  turnSensorReset();
  turnAngleInDegrees = (((int32_t)turnAngle >> 16) * 360) >> 16;
}




