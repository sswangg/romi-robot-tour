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

#define NIGHTY_LEFT_TURN_COUNT -709
#define NIGHTY_RIGHT_TURN_COUNT 708



char moves[200] = "S L F100 R F B E";
double targetTime = 65;
double endDist = 41;
double startDist = -16;



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

void loop() {
  unsigned long endTime;
  if (buttonA.getSingleDebouncedPress()) {
    delay(300);
    robotState = ROBOT_MOVE;
  }
  if (robotState == ROBOT_MOVE) {
    int count = 1;
    for (int i = 0; i < strlen(moves); i++)
      if (isSpace(moves[i])) count++;
      
    char *movesList[count];
    char *ptr = NULL;

    byte index = 0;
    ptr = strtok(moves, " ");
    while (ptr != NULL) {
      movesList[index] = ptr;
      index++;
      ptr = strtok(NULL, " ");
    }

    int numTurns = 0;
    double totalDist = 0;
    char currentChar;
    int currentLen;
    String st;

    for (int i = 0; i < count; i++) {
      currentChar = *movesList[i];
      st = movesList[i];
      if (currentChar == 'R' || currentChar == 'L') {
        numTurns++;
      }
      else if (currentChar == 'F' || currentChar == 'B') {   
        if (st.length() > 1) {
          totalDist += st.substring(1).toDouble();
        } else {
          totalDist += 50;
        }
      } else if (currentChar == 'S') {
        totalDist += abs(startDist);
      } else if (currentChar == 'E') {
        totalDist += abs(endDist);
      }
    }

    double turnTime = 0.55;
    double totalTurnTime = 0.65 * numTurns;
    double totalDriveTime = targetTime - totalTurnTime - 0.004*totalDist;
    double dist;

    for (int i = 0; i < count; i++) {
      currentChar = *movesList[i];
      st = movesList[i];

      if (currentChar == 'R') {
        right(turnTime);
      } else if (currentChar == 'L') {
        left(turnTime);
      }
      else if (currentChar == 'F' || currentChar == 'B') {      
        if (st.length() > 1) {
          dist = st.substring(1).toDouble();
        } else {
          dist = 50;
        }
        if (currentChar == 'F') {
          chassis.driveWithTime(dist, dist/totalDist * totalDriveTime);
        } else {
          chassis.driveWithTime(0-dist, dist/totalDist * totalDriveTime);
        } 
      } else if (currentChar == 'S') {
        chassis.driveWithTime(startDist, abs(startDist)/totalDist * totalDriveTime);
      } else if (currentChar == 'E') {
        chassis.driveWithTime(endDist, abs(endDist)/totalDist * totalDriveTime);
      }
    }
    robotState = ROBOT_IDLE;
  }

  oled.setRow(0);
  //oled.print(String(chassis.leftMotor.getCount()) + ", " + String(chassis.rightMotor.getCount()));
  oled.clearToEOL();
  oled.println();
  oled.clearToEOL();
  oled.println();
}
