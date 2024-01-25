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

SSD1306AsciiAvrI2c oled;

// Declare a chassis object with nominal dimensions
// In practice, adjust the parameters: wheel diam, encoder counts, wheel track
Chassis chassis(6.994936972, 1440, 14.0081);
Romi32U4ButtonA buttonA;

// Define the states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_MOVE, MOVING};
ROBOT_STATE robotState = ROBOT_IDLE;

// A helper function to stop the motors
void idle(void)
{
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
void setup() 
{
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
}

void turnLeft() {
  chassis.turnFor(88, 60);
  delay(100);
}

void turnRight() {
  chassis.turnFor(-88, 60);
  delay(100);
}

/*
 * The main loop for the program. The loop function is repeatedly called
 * after setup() is complete.
 */
float targetTime = 63;
float numTurns = 9;
float legs = 14;
// time per turn around 1.9, but would rather be over than under
float legTime = (targetTime - numTurns * 1.8)/legs;
void loop()
{
  unsigned long startTime, endTime;
  if (buttonA.getSingleDebouncedPress()) {
    delay(500);
    robotState = ROBOT_MOVE;
  }
  if (robotState == ROBOT_MOVE) {
    //chassis.driveFor(30, 55);
    //chassis.driveForTurn(54.9715, 76.9754); // 28.2
    //chassis.driveFor(30, 55);
    startTime = millis();
    chassis.driveWithTime(100, 2);
    endTime = millis();
    robotState = ROBOT_IDLE;
  }

  oled.setRow(0);
  //oled.print(String(chassis.leftMotor.getCount()) + ", " + String(chassis.rightMotor.getCount()));
  oled.print(String(endTime-startTime));
  oled.clearToEOL();
  oled.println();
  oled.setRow(1);
  oled.clearToEOL();
  oled.println();
}
