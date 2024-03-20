/**
 * Demonstration of controlling two servos simultanesouly. See documentation for servo.h/.cpp for
 * more details.
 * 
 * */

#include <Arduino.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>
#include "Chassis.h"
#include "Romi32U4Buttons.h"

#define I2C_ADDRESS 0x3C

#define NIGHTY_LEFT_TURN_COUNT -722
#define NIGHTY_RIGHT_TURN_COUNT 722

SSD1306AsciiAvrI2c oled;

// Declare a chassis object with nominal dimensions
// In practice, adjust the parameters: wheel diam, encoder counts, wheel track
Chassis chassis(6.994936972, 1440, 14.0081);
Romi32U4ButtonA buttonA;

// Define the states
enum ROBOT_STATE { ROBOT_IDLE,
                   ROBOT_MOVE,
                   MOVING };
ROBOT_STATE robotState = ROBOT_IDLE;

// A helper function to stop the motors
void idle(void) {
  Serial.println("idle()");

  //stop motors
  chassis.idle();

  //set state to idle
  robotState = ROBOT_IDLE;
}


/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup() {
  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);
  // Serial1 is used to receive data from K210

  // initialize the chassis (which also initializes the motors)
  chassis.init();
  idle();

  // these can be undone for the student to adjust
  chassis.setMotorPIDcoeffs(5, 0.5);

  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(System5x7);
  oled.clear();
  oled.print("Hello world!");
  delay(1000);
  oled.clear();
}

void turnLeft() {
  chassis.turnFor(89, 60);
  delay(100);
}

void turnRight() {
  chassis.turnFor(-89, 60);
  delay(100);
}

void left(float seconds) {
  //long oldCount = chassis.leftMotor.getCount();
  chassis.turnWithTimePosPid(NIGHTY_LEFT_TURN_COUNT, seconds);
  //long newCount = chassis.leftMotor.getCount();

  oled.println(String(chassis.rightMotor.getCount()) + " - " + String(chassis.leftMotor.getCount()));
  
  
  // oled.setRow(0);
  // oled.clearToEOL();
  // oled.println();
}

void right(float seconds) {
  chassis.turnWithTimePosPid(NIGHTY_RIGHT_TURN_COUNT, seconds);
}

// TO USE PROGRAM
// Set target time to the target time minus 1
// Manually count the number of turns and legs
// That's it
float targetTime = 55;
float numTurns = 13;
float legs = 28;
float legTime = ((targetTime - 1.5) - numTurns * 0.9) / legs;
float t = 0.8;

//for final dist timing
unsigned long startTime;
void loop() {
  unsigned long endTime;
  if (buttonA.getSingleDebouncedPress()) {
    delay(500);
    startTime = millis();
    robotState = ROBOT_MOVE;
  }
  if (robotState == ROBOT_MOVE) {
    chassis.driveWithTime(25+8.5, legTime);
    right(.8);
    chassis.driveWithTime(50, legTime);
    left(.8);
    chassis.driveWithTime(50, legTime);
    left(.8);
    chassis.driveWithTime(50, legTime);
    right(.8);
    chassis.driveWithTime(50, legTime);
    right(.8);
    chassis.driveWithTime(50, legTime);
    left(.8);
    chassis.driveWithTime(50, legTime);
    left(.8);
    chassis.driveWithTime(50, legTime);
    chassis.driveWithTime(-50, legTime);
    left(.8);
    chassis.driveWithTime(50, legTime);
    right(.8);
    chassis.driveWithTime(50, legTime);
    left(.8);
    chassis.driveWithTime(50, legTime);
    left(.8);
    chassis.driveWithTime(50, legTime);
    right(.8);
    chassis.driveWithTime(50, legTime);
    right(.8);
    chassis.driveWithTime(100, legTime*2);

    right(.8);
    chassis.driveWithTime(50, legTime);
    left(.8);
    chassis.driveWithTime(50, legTime);
    right(.8);
    chassis.driveWithTime(50, legTime);
    right(.8);
    chassis.driveWithTime(50, legTime);
    left(.8);
    chassis.driveWithTime(50, legTime);
    left(.8);
    chassis.driveWithTime(50, legTime);
    left(.8);
    chassis.driveWithTime(150, legTime*2);
    chassis.driveWithTime(-5/0, legTime);
    left(.8);
    chassis.driveWithTime(50, legTime);
    right(.8);
    chassis.driveWithTime(50, legTime);
    left(.8);
    chassis.driveWithTime(50-8.5, legTime);

    robotState = ROBOT_IDLE;
  }

  // oled.setRow(0);
  // oled.print(String(chassis.leftMotor.getCount()) + ", " + String(chassis.rightMotor.getCount()));
  // oled.clearToEOL();
  // oled.println();
  // oled.setRow(1);
  // oled.clearToEOL();
  // oled.println();
}
