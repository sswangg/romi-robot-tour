// Adapted from a library by Pololu Corporation.  For more information, see http://www.pololu.com/

#include "Romi32U4Motors.h"
#include "FastGPIO.h"
#include <avr/io.h>

// define the motor pins here
#define PWM_L 10
#define PWM_R 9
#define DIR_L 16
#define DIR_R 15

/**
 * initMotors() should be called near the beginning of the program (usually in Chassis::init()).
 * It sets up Timer4 to run at 38 kHz, which is used to both drive the PWM signal for the motors
 * and (tangentially) allow for a 38 kHz signal on pin 11, which can be used, say, to drive an
 * IR LED at a common rate.
 * 
 * Timer 1 has the following configuration:
 *  prescaler of 1
 *  outputs enabled on channels A (pin 9), B (pin 10) and C (pin 11)
 *  fast PWM mode
 *  top of 420, which will be the max speed
 *  frequency is then: 16 MHz / [1 (prescaler) / (420 + 1)] = 38.005 kHz
 * */
void Romi32U4Motor::initMotors() {
  Serial.println("initMotors()");

  FastGPIO::Pin<PWM_L>::setOutputLow();
  FastGPIO::Pin<PWM_R>::setOutputLow();
  FastGPIO::Pin<DIR_L>::setOutputLow();
  FastGPIO::Pin<DIR_R>::setOutputLow();

  noInterrupts();  //disable interupts while we set Timer1 registers

  TCCR1A = 0xAA;  //0b10101010; //Fast PWM + outputs enabled
  TCCR1B = 0x19;  //0b00011001; //Fast PWM
  ICR1 = 420;     //runs at 38kHz; lowers speed for given effort by 5% from Pololu version

  //set all three outputs to zero
  OCR1A = 0;
  OCR1B = 0;
  OCR1C = 0;  //can be used to create 38 kHz signal on pin 11

  interrupts();  //re-enable interrupts

  Serial.println("/initMotors()");
}

/**
 * Because the Pololu library is based on the FastGPIO library, we don't/can't use analogWrite.
 * Instead, we set the duty cycle directly at the register level.
 * 
 * We also have to have separate classes for the left and right motors to avoid the complex
 * mapping of speeds to registers.
 * */
void LeftMotor::setEffort(int16_t effort) {
  bool reverse = 0;

  if (effort < 0) {
    effort = -effort;  // Make speed a positive quantity.
    reverse = 1;       // Preserve the direction.
  }
  if (effort > maxEffort) {
    effort = maxEffort;
  }

  OCR1B = effort;
  FastGPIO::Pin<DIR_L>::setOutput(reverse);
}

void RightMotor::setEffort(int16_t effort) {
  bool reverse = 0;

  if (effort < 0) {
    effort = -effort;  // Make speed a positive quantity.
    reverse = 1;       // Preserve the direction.
  }
  if (effort > maxEffort) {
    effort = maxEffort;
  }

  OCR1A = effort;
  FastGPIO::Pin<DIR_R>::setOutput(reverse);
}

/**
 * Top speed is limited to 300/420 by default. This allow you to go faster. Be careful.
 * */
void Romi32U4Motor::allowTurbo(bool turbo) {
  maxEffort = turbo ? 400 : 300;
}

/**
 * update() must be called regularly to update the control signals sent to the motors.
 * */
void Romi32U4Motor::update(void) {
  if (ctrlMode == CTRL_SPEED || ctrlMode == CTRL_POS) {
    int16_t effort = pidSpeedCtrl.calcEffort(targetSpeed - speed);
    setEffort(effort);
  } else if (ctrlMode == CTRL_PIDPOS) {
    int16_t effort = pidPosCtrl.calcEffort(targetCount - count);
    if (effort > 0) {
      if (effort > maxTurnEffort)
        effort = maxTurnEffort;
      else if (effort < minEffort && pidPosCtrl.currError != 0)
        effort = minEffort;
    } else if (effort < 0) {
      if (effort < -maxTurnEffort)
        effort = -maxTurnEffort;
      else if (effort > -minEffort && pidPosCtrl.currError != 0)
        effort = -minEffort;
    }
    setEffort(effort);
  }
}

/**
 * Sets the target speed in "encoder ticks/16 ms interval"
 * */
void Romi32U4Motor::setTargetSpeed(float target) {
  targetSpeed = target;

  if (ctrlMode != CTRL_SPEED) {
    // Reset the error integral if we are switching from another mode
    // Otherwise, the robot may jump due to residual integral
    pidSpeedCtrl.resetSum();

    //Also reset prevCount to avoid jumps
    prevCount = count;
  }

  ctrlMode = CTRL_SPEED;
}

/**
 * Sets the target encoder count
 * */
void Romi32U4Motor::setTargetCount(int target) {
  cli();
  count = 0;
  prevCount = 0;
  sei();
  targetCount = target;
  ctrlMode = CTRL_PIDPOS;
}

/**
 * Sets the (delta) target position in "encoder ticks" and a speed to drive to get there
 * in "encoder ticks/16 ms interval"
 * */
void Romi32U4Motor::moveFor(long amount) {
  cli();
  long currPos = count;
  sei();

  targetPos = currPos + amount;
  ctrlMode = CTRL_POS;
}
