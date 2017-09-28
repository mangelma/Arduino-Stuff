// Team 14
// ME311 Dead Reckoning assignment

#include <Romi32U4.h>
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Romi32U4LCD lcd;
Romi32U4Motors motors;
Romi32U4Buzzer buzzer;
Romi32U4Encoders encoders;
int16_t countsLeft = 0;
int16_t countsRight = 0;
int delayBetweenMoves = 100;
String debug = "OFF";

void setup() {
  lcd.print("14 Ready");
} // end of setup

void loop()
{
  if (buttonA.isPressed()) {
    launchingSequence();
    navigateThroughTrack();
  }

// testing and calibration functions
  if (buttonB.isPressed()) {
    lcd.clear();
    lcd.write("C pressed");
    delay(1000);
    goStraight(28.5, 75);
  }

  if (buttonC.isPressed()) {
    lcd.clear();
    lcd.write("C pressed");
    delay(1000);
    lcd.clear();
    movingTurn(6.44, 1.96, 75); // right
    delay(1000);
    movingTurn(1.96, 6.44, 75); // left
  }

}

void stopMotors() {
  int speed = 0;
  motors.setLeftSpeed(speed);
  motors.setRightSpeed(speed);
}

void launchingSequence() {
    lcd.clear();
    lcd.write("going in");
    delay(500);
    lcd.clear();
    lcd.write("3");
    buzzer.playFrequency(800, 100, 15);
    delay(500);
    lcd.clear();
    lcd.write("2");
    buzzer.playFrequency(800, 100, 15);
    delay(500);
    lcd.clear();
    lcd.write("1");
    buzzer.playFrequency(800, 100, 15);
    delay(500);
}

void navigateThroughTrack() {

    buzzer.playFrequency(800, 200, 15);
    goStraight(12.5, 75);
    buzzer.playFrequency(800, 200, 15);
    delay(delayBetweenMoves);

    // first turn
    buzzer.playFrequency(800, 200, 15);
    stationaryRightTurn(4.42, 75);
    buzzer.playFrequency(800, 200, 15);
    delay(delayBetweenMoves);

    // going straight right past the first obstacle
    buzzer.playFrequency(800, 200, 15);
    goStraight(10, 75);
    buzzer.playFrequency(800, 200, 15);
    delay(delayBetweenMoves);

    // 5" rad. turn to right
    buzzer.playFrequency(800, 200, 15);
    movingTurn(6.44, 1.96, 75);
    buzzer.playFrequency(800, 200, 15);
    delay(delayBetweenMoves);

    // going straight past the second obstacle
    buzzer.playFrequency(800, 200, 15);
    goStraight(17, 75);
    buzzer.playFrequency(800, 200, 15);
    delay(delayBetweenMoves);

    // another 5" turn but left
    buzzer.playFrequency(800, 200, 15);
    movingTurn(1.96, 6.44, 75);
    buzzer.playFrequency(800, 200, 15);
    delay(delayBetweenMoves);

    // straight until we are at final X coordinate
    buzzer.playFrequency(800, 200, 15);
    goStraight(28.5, 75); 
    buzzer.playFrequency(800, 200, 15);
    delay(delayBetweenMoves);

    // left 90 degree turn 
    buzzer.playFrequency(800, 200, 15);
    stationaryLeftTurn(4.42, 75);
    buzzer.playFrequency(800, 200, 15);
    delay(delayBetweenMoves);

    // TO THE LANDING STONE
    buzzer.playFrequency(800, 200, 15);
    goStraight(12.5, 75);
    buzzer.playFrequency(800, 200, 15);
    delay(500);

    // display stopped
    lcd.clear();
    lcd.write("STOPPED");
}

void goStraight(float distance, int speed) {

  countsLeft = 0;
  countsRight = 0;
  float correction = 5; // plus to right speed
  int countsLeftBefore = encoders.getCountsLeft();
  int countsRightBefore = encoders.getCountsRight();
  float leftTarget = distance * 161.2;
  float rightTarget = distance * 161.2;

  // printing our distance to screen
  lcd.clear();
  lcd.print(distance);

  // running motors until either targets is hit
  while (countsLeft <= leftTarget && countsRight <= rightTarget) {

    // set the motors speeds
    motors.setLeftSpeed(speed);
    motors.setRightSpeed(speed + correction);

    // calculate the actual distance in counts
    int leftAfter = encoders.getCountsLeft();
    int rightAfter = encoders.getCountsRight();
    countsLeft = leftAfter - countsLeftBefore;
    countsRight = rightAfter - countsRightBefore;

  }
  
  stopMotors();

}

void stationaryRightTurn(float distance, int speed) {

  countsLeft = 0;
  countsRight = 0;
  int correction = 3;
  float leftTarget = distance * 161.2;
  float rightTarget = distance * -161.2;
  int countsLeftBefore = encoders.getCountsLeft();
  int countsRightBefore = encoders.getCountsRight();

  lcd.clear();
  lcd.print(leftTarget);
  lcd.gotoXY(0, 1);
  lcd.print(rightTarget);

   // running motors until either targets is hit
  while (countsLeft <= leftTarget && countsRight >= rightTarget) {

    // set the motors speeds
    motors.setLeftSpeed(speed);
    motors.setRightSpeed(-speed + correction);

    // calculate the actual distance in counts
    int leftAfter = encoders.getCountsLeft();
    int rightAfter = encoders.getCountsRight();
    countsLeft = leftAfter - countsLeftBefore; // goes below zero
    countsRight = rightAfter - countsRightBefore; // should stay under

  }

  stopMotors();
}

void stationaryLeftTurn(float distance, int speed) {

  countsLeft = 0;
  countsRight = 0;
  int correction = 3;
  float leftTarget = distance * -161.2;
  float rightTarget = distance * 161.2;
  
  lcd.clear();
  lcd.print(leftTarget);
  lcd.gotoXY(0, 1);
  lcd.print(rightTarget);

  // get the count values from encoders
  int countsLeftBefore = encoders.getCountsLeft();
  int countsRightBefore = encoders.getCountsRight();
  Serial.println(countsLeftBefore);
  Serial.println(countsRightBefore);
  Serial.println(countsLeft);
  Serial.println(countsRight);

   // running motors until either targets is hit
  while (countsLeft >= leftTarget && countsRight <= rightTarget) {

    // set the motors speeds
    motors.setLeftSpeed(-speed);
    motors.setRightSpeed(speed + correction);

    // measureing how many counts we have
    int16_t leftAfter = encoders.getCountsLeft();
    int16_t rightAfter = encoders.getCountsRight();
    countsLeft = leftAfter - countsLeftBefore; // goes below zero
    countsRight = rightAfter - countsRightBefore; // should stay under

  }
 stopMotors();
}

void movingTurn(float leftDistance, float rightDistance, int speed) {

  int leftSpeed;
  int rightSpeed;
  int leftCorr = 5; // used for tuning 
  int rightCorr = 10; // used for tuning
  float leftTarget;
  float rightTarget;
  int countsLeftBefore;
  int countsRightBefore;
  float calibValue = 161.2; // for motor speed of 75
  countsLeft = 0;
  countsRight = 0;

  // turning right
  if (leftDistance > rightDistance) {
    leftSpeed = speed;
    rightSpeed = speed * (rightDistance/leftDistance) + rightCorr;
  } 

  // turning left
  else if (leftDistance < rightDistance) {
    leftSpeed = speed * (leftDistance/rightDistance) + leftCorr;
    rightSpeed = speed;
  }

  leftTarget = leftDistance * calibValue;
  rightTarget = rightDistance * calibValue;
  countsLeftBefore = encoders.getCountsLeft();
  countsRightBefore = encoders.getCountsRight();

  lcd.clear();
  lcd.print(leftTarget);
  lcd.gotoXY(0, 1);
  lcd.print(rightTarget);

  // running motors until either targets is hit
  while (countsLeft <= leftTarget && countsRight <= rightTarget) {

    // set the motors speeds
    motors.setLeftSpeed(leftSpeed);
    motors.setRightSpeed(rightSpeed);

    // calculate the actual distance in counts
    int16_t leftAfter = encoders.getCountsLeft();
    int16_t rightAfter = encoders.getCountsRight();
    countsLeft = leftAfter - countsLeftBefore;
    countsRight = rightAfter - countsRightBefore;

  }

  if (debug == "ON") {
    lcd.print(countsLeft);
    lcd.print(" ");
    lcd.print(countsRight);
    lcd.gotoXY(0, 1);
  }

 stopMotors();

}

