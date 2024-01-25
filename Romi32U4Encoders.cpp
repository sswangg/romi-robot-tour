// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/

#include <Arduino.h>
#include "Romi32U4Motors.h"
#include "FastGPIO.h"
#include <avr/interrupt.h>

#include "pcint.h"

#include "Chassis.h"

#define LEFT_XOR   8
#define LEFT_B     IO_E2
#define RIGHT_XOR  7
#define RIGHT_B    23

// declared here to keep the compiler happy; defined at the bottom
void leftISR(void);
void rightISR(void);

/**
 * Set up the encoder 'machinery'. Call it near the beginning of the program.
 * 
 * Do not edit this function.
 * */
void Romi32U4Motor::initEncoders(void)
{    
    Serial.println("initEncoders()");
    // Set the pins as pulled-up inputs.
    FastGPIO::Pin<LEFT_XOR>::setInputPulledUp();
    FastGPIO::Pin<LEFT_B>::setInputPulledUp();
    FastGPIO::Pin<RIGHT_XOR>::setInputPulledUp();
    FastGPIO::Pin<RIGHT_B>::setInputPulledUp();

    //attach a PC interrupt
    attachPCInt(PCINT4, leftISR); // pin change interrupts fire on CHANGE

    // Enable interrupt on PE6 for the right encoder.  We use attachInterrupt
    // instead of defining ISR(INT6_vect) ourselves so that this class will be
    // compatible with other code that uses attachInterrupt.
    attachInterrupt(4, rightISR, CHANGE);

    // Initialize the variables.  It's good to do this after enabling the
    // interrupts in case the interrupts fired as we were enabling
    // them.
    bool lastLeftB = FastGPIO::Pin<LEFT_B>::isInputHigh();
    bool lastLeftA = FastGPIO::Pin<LEFT_XOR>::isInputHigh() ^ lastLeftB;

    chassis.leftMotor.handleISR(lastLeftA, lastLeftB);
    chassis.leftMotor.getAndResetCount();

    bool lastRightB = FastGPIO::Pin<LEFT_B>::isInputHigh();
    bool lastRightA = FastGPIO::Pin<LEFT_XOR>::isInputHigh() ^ lastRightB;

    chassis.rightMotor.handleISR(lastRightA, lastRightB);
    chassis.rightMotor.getAndResetCount();

    Serial.println("/initEncoders()");
}

/**
 * Returns the current encoder count.
 * */
long Romi32U4Motor::getCount()
{
    cli();
    long tempCount = count;
    sei();
    return tempCount;
}

/**
 * Resets the encoder count and returns the last count.
 * */
long Romi32U4Motor::getAndResetCount(void)
{
    cli();
    long tempCount = count;
    count = 0;
    prevCount = 0;
    sei();
    return tempCount;
}

/**
 * calcEncoderDelta() is called automatically by an ISR. It takes a 'snapshot of the encoders and 
 * stores the change since the last call in speed, which has units of "encoder ticks/16 ms interval" 
 * 
 * Because it is called from within an ISR, interrupts don't need to be disabled.
 * */
void Romi32U4Motor::calcEncoderDelta(void) 
{
    long currCount = count;
    speed = currCount - prevCount;
    prevCount = currCount;
}

/**
 * Calculates the encoder counter increment/decrement due to an encoder transition. Pololu sets
 * up their encoders in an interesting way with some logic chips, so first we have to deconvolute
 * the encoder signals (in the ISR); then, we call this function to  update the counter. 
 * 
 * More details are found here:
 * 
 * https://www.pololu.com/docs/0J69/3.3
 * 
 * This function is called from the ISR, which does the actual deconvolution for each motor.
 * */
void Romi32U4Motor::handleISR(bool newA, bool newB)
{
    count += (lastA ^ newB) - (newA ^ lastB);

    lastA = newA;
    lastB = newB;

    if(ctrlMode == CTRL_POS)
    {
        if(count == targetPos)
        {
            setEffort(0);
            ctrlMode = CTRL_DIRECT;
        }
    }
}

/**
 * ISR to update the encoder counter for the left motor.
 * */
void leftISR()
{
    bool newLeftB = FastGPIO::Pin<LEFT_B>::isInputHigh();
    bool newLeftA = FastGPIO::Pin<LEFT_XOR>::isInputHigh() ^ newLeftB;

    chassis.leftMotor.handleISR(newLeftA, newLeftB);
}

/**
 * ISR to update the encoder counter for the left motor.
 * */
void rightISR()
{
    bool newRightB = FastGPIO::Pin<RIGHT_B>::isInputHigh();
    bool newRightA = FastGPIO::Pin<RIGHT_XOR>::isInputHigh() ^ newRightB;

    chassis.rightMotor.handleISR(newRightA, newRightB);
}