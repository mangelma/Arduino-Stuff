#include <Wire.h>
#include <Romi32U4.h>
#include <LSM6.h>
Romi32U4Motors motors;
Romi32U4LCD lcd;
Romi32U4ButtonA buttonA;
Romi32U4Encoders encoders;
Romi32U4Buzzer buzzer;
LSM6 imu;

const int16_t maxSpeed = 150;
int stopFaceUp = 200;             // when i increments to this value, stopping begins
const int numReadings = 10;       // number of angle readings 
int readings[numReadings];        // the readings stored from accelerometer
int readingsY[numReadings];       // the readings stored from accelerometer
int readIndex = 0;                // the index of the current reading
int readIndexY = 0;               // the index of the current reading
long total = 0;                   // the running total
long totalY = 0; 
int averageX = 0;                 // the average
int averageY = 0;                 // the average
float xOffset = 0;                // calibration value
float yOffset = 0;                // calibration value
int i = 0;                        // incrementing value for detecting when facing uphill
int16_t x;                        // actual x reading from accelerometer
int16_t y;                        // actual y reading from accelerometer
int32_t magnitudeSquared;         // from faceUpHill example
float xAngle;                     // averaged value from x reading

void setup() {
  // initialize our array to store x values
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
      readings[thisReading] = 0;
      readingsY[thisReading] = 0;
    }
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b10001000);
  imu.writeReg(LSM6::CTRL1_XL, 0b10000100);
  lcd.clear();
  lcd.print(F("calib."));
  calibrateXY();
  lcd.clear();
  lcd.print(F("ramp, A"));
  buttonA.waitForPress();
  delay(500);
  lcd.clear();
}

void loop() {
  updateXY();
  int16_t forwardSpeed = -(encoders.getCountsLeft() + encoders.getCountsRight());
  forwardSpeed = constrain(forwardSpeed, -maxSpeed, maxSpeed);
  y = smoothingY(y-yOffset);
  int16_t turnSpeed = y;
  int16_t leftSpeed = forwardSpeed - turnSpeed;
  int16_t rightSpeed = forwardSpeed + turnSpeed;
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);
  motors.setSpeeds(leftSpeed, rightSpeed);

  // we are stopped when turnspeed is low enough and x is positive
  // may need adjustment, 107 is 3 degrees where we aim
  if (y < 107 && y > -107 && x > 0) {
    i++;
    lcd.gotoXY(0,0);
    lcd.print(y);
    lcd.gotoXY(0,1);
    lcd.print(x);
  }

 // when i is incremented enough, we save the angle
 if (i == stopFaceUp) {
    int timeStopped = 0;
    buzzer.playFrequency(800, 100, 10);
    motors.setSpeeds(0,0); // STOP
    int stopStartTime = millis(); 
    while(timeStopped < 3000) {
      xAngle = smoothing(x);
      updateXY();
      int stopStopTime = millis();
      timeStopped = stopStopTime-stopStartTime;
    }
    lcd.gotoXY(0, 0);
    lcd.print((asin( (xAngle - xOffset)/2048) )/(3.141/180)); // accelerometer units to degrees
    lcd.gotoXY(0,1);
    lcd.print(timeStopped);
    goUpHill(); // time to go
  }
}

void calibrateXY() { // called from setup()
    updateXY();
    for (int i; i<500; i++) {
       xOffset = smoothing(x);
       yOffset = smoothingY(y);
       xOffset = constrain(xOffset, -50, 50); // don't want to calibrate too much
       yOffset = constrain(yOffset, -50, 50); // don't want to calibrate too much
       lcd.gotoXY(0,0);
       lcd.print(xOffset);
       lcd.gotoXY(0,1);
       lcd.print(yOffset);
    }
  delay(500); // display offsets
}

void updateXY() { // from example faceuphill
  imu.read();
  x = imu.a.x;
  y = imu.a.y;
  magnitudeSquared = (int32_t)x * x + (int32_t)y * y;
}

float smoothing(int xValue) { // from arduino.cc, trying to filter noise by averaging
  total = total - readings[readIndex];
  readings[readIndex] = xValue;
  //Serial.println(xValue);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  averageX = total / numReadings;
  return averageX;
}

float smoothingY(int yValue) { // from arduino.cc, trying to filter noise by averaging
  totalY = totalY - readingsY[readIndexY];
  readingsY[readIndexY] = yValue;
  totalY = totalY + readingsY[readIndexY];
  readIndexY = readIndexY + 1;
  if (readIndexY >= 10) {
    readIndexY = 0;
  }
  averageY = totalY / 10;
  return averageY;
}

void goUpHill() { // let's go
  int j = 0;
  while(j < 30) { // adjust if needed, smaller value stops the robot sooner 
     motors.setSpeeds(150,150);
     updateXY();
     if (x < 0) {
      j++; // for redudancy; one negative value can't stop us from getting to top!
     }
  }
    delay(100); // and make sure we're past the corner of the ramp
    motors.setSpeeds(0,0); // STOP
    buttonA.waitForPress(); // not going anywhere unless button a is pressed again
}
