#include <Wire.h>
#include <Romi32U4.h>
#include <LSM6.h>
Romi32U4Motors motors;
Romi32U4LCD lcd;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB   buttonB;
Romi32U4Encoders encoders;
Romi32U4Buzzer buzzer;
LSM6 imu;
const int16_t maxSpeed = 100;
int startMeasure = 250; // when to start measuring
int startClimb = 750;   // when to start driving up the ramp
//const int numReadings = startClimb - startMeasure; http://forum.arduino.cc/index.php?topic=287019.0
const int numReadings = 500;      // number of angle readings 
int readings[numReadings];        // the readings stored from accelerometer
int readIndex = 0;                // the index of the current reading
long total = 0;                   // the running total
int average = 0;                  // the average
float xOffset = 0;                // calibration value
int i = 0;                        // incrementing value for detecting when facing uphill
int16_t x;                        // actual x reading from accelerometer
int16_t y;                        // actual y reading from accelerometer
int32_t magnitudeSquared;         // from faceUpHill example
float xAngle;                     // averaged value from x reading

void calibrateX() {
    updateXY();
    for (int i; i<500; i++) {
       xOffset = smoothing(x);
       lcd.gotoXY(0,0);
       lcd.print(xOffset);
    }
  delay(500);
}

void updateXY() {
  // Read the acceleration from the LSM6DS33.
  // A value of 2048 corresponds to approximately 1 g.
  imu.read();
  x = imu.a.x;
  y = imu.a.y;
  magnitudeSquared = (int32_t)x * x + (int32_t)y * y;
}

float smoothing(int xValue) { // from arduino.cc
  total = total - readings[readIndex];
  readings[readIndex] = xValue;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  average = total / numReadings;
  return average;
}

void goUpHill() {
  while(x > 0) {
     motors.setSpeeds(95,100);
     updateXY();
     delay(100);
  }
    motors.setSpeeds(0,0);
    i = 0;
    buttonA.waitForPress();
}

void setup()
{
  // initialize our array to store x values
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
      readings[thisReading] = 0;
    }
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b10001000);
  imu.writeReg(LSM6::CTRL1_XL, 0b10000100);
  lcd.clear();
  lcd.print(F("calib."));
  calibrateX();
  lcd.clear();
  lcd.print(F("Press A"));
  buttonA.waitForPress();
  delay(500);
  lcd.clear();
}

void displayStuffOnLCD() {
  static uint8_t lastDisplayTime;
  if ((uint8_t)(millis() - lastDisplayTime) > 150)
  {
    lastDisplayTime = millis();
    lcd.gotoXY(0, 0);
    lcd.print(xAngle/22.75 - xOffset); // 22.75 is 2048 units of accelerometer values divided by 90 degrees
    lcd.print(F("       "));
    lcd.gotoXY(0, 1);
    lcd.print(x);
    lcd.print(F("       "));
  }
}

void loop()
{
  updateXY();
  displayStuffOnLCD();
  int16_t forwardSpeed = -(encoders.getCountsLeft() + encoders.getCountsRight());
  forwardSpeed = constrain(forwardSpeed, -maxSpeed, maxSpeed);

  // See if we are actually on an incline.
  // 2048 * sin(5 deg) = 178
  int16_t turnSpeed;
  if (magnitudeSquared > (int32_t)178 * 178)
  { turnSpeed = y; }
  else { turnSpeed = 0; }

  // we are stopped when turnspeed is low enough and x is positive
  // may need adjustment
  if (turnSpeed < 25 && turnSpeed > -25 && x > 0) {
    i++;
    if (i > startMeasure) {   // when some time is passed, start measuring angle
      xAngle = smoothing(x);  // smooth the x value and save it to xAngle
    }
  }
 
  int16_t leftSpeed = forwardSpeed - turnSpeed;
  int16_t rightSpeed = forwardSpeed + turnSpeed;
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);
  motors.setSpeeds(leftSpeed, rightSpeed);
  
 if (i == startClimb) {
    Serial.println("going up!");
    lcd.gotoXY(0, 0);
    lcd.print(xAngle/22.75);
    goUpHill();
  }

}
