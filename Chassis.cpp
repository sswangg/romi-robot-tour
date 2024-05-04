#include "Chassis.h"


float calculateSpeed(float forwardDistance, float targetSeconds, float elapsedSeconds, float minSpeed = 15) {
  float speedMultiplier = 1.05;
  if (forwardDistance < 0) speedMultiplier = -1.05;

  float percentComplete = elapsedSeconds / targetSeconds;
  float delta = (fabs(forwardDistance) - minSpeed * targetSeconds) / (0.6 * targetSeconds); // diff between max and min speed
  float s = delta + minSpeed; // s is max speed if it's not at beginning or end

  // if in first or last 40%, calculate speed using trapezoidal motion profile
  if (percentComplete < 0.4) {
    s = delta * percentComplete / 0.4 + minSpeed;
  } else if (percentComplete > 0.6) {
    s = delta * (1 - percentComplete) / 0.4 + minSpeed;
  }

  return s * speedMultiplier;  // tends to be a little slow, friction and stuff ig
}

int calculateIntermediateTargetLinear(int target, float finishSeconds, float elapsedSeconds) {
  if (elapsedSeconds >= finishSeconds) return target;
  return int(elapsedSeconds * (float(target) / finishSeconds));
}

/**
 * Call init() in your setup() routine. It sets up some internal timers so that the speed controllers
 * for the wheels will work properly.
 * 
 * Here's how it works: Motor::init() starts a hardware timer on a 16 ms loop. Every time the timer 
 * "rolls over," an interrupt service routine (ISR) is called that updates the motor speeds and 
 * sets a flag to notify Chassis that it is time to calculate the control inputs.
 * 
 * When set up this way, pins 6, 12, and 13 cannot be used with analogWrite()
 * */
void Chassis::init(void) {
  Romi32U4Motor::init();

  // temporarily turn off interrupts while we set the time up
  noInterrupts();

  // sets up timer 4 for a 16 ms loop, which triggers the motor PID controller
  // dt = 1024 (prescaler) * (249 + 1) / 16E6 (clock speed) = 16 ms
  TCCR4A = 0x00;  //disable some functionality
  TCCR4B = 0x0B;  //sets the prescaler to 1024
  TCCR4C = 0x00;  //disable outputs (overridden for the servo class)
  TCCR4D = 0x00;  //fast PWM mode (for servo)

  OCR4C = 249;  //TOP goes in OCR4C

  TIMSK4 = 0x04;  //enable overflow interrupt

  // re-enable interrupts
  interrupts();
}

void Chassis::setMotorPIDcoeffs(float kp, float ki) {
  leftMotor.setSpeedPIDCoeffients(kp, ki);
  rightMotor.setSpeedPIDCoeffients(kp, ki);
}

/**
 * Stops the motors. It calls setMotorEfforts() so that the wheels won't lock. Use setSpeeds() if you want 
 * the wheels to 'lock' in place.
 * */
void Chassis::idle(void) {
  setMotorEfforts(0, 0);
}

/**
 * Sets the motor _efforts_.
 * */
void Chassis::setMotorEfforts(int leftEffort, int rightEffort) {
  leftMotor.setMotorEffort(leftEffort);
  rightMotor.setMotorEffort(rightEffort);
}

void Chassis::setWheelSpeeds(float leftSpeed, float rightSpeed) {
  int16_t leftTicksPerInterval = (leftSpeed * (ctrlIntervalMS / 1000.0)) / cmPerEncoderTick;
  int16_t rightTicksPerInterval = (rightSpeed * (ctrlIntervalMS / 1000.0)) / cmPerEncoderTick;

  leftMotor.setTargetSpeed(leftTicksPerInterval);
  rightMotor.setTargetSpeed(rightTicksPerInterval);
}

void Chassis::setTwist(float forwardSpeed, float turningSpeed) {
  int16_t ticksPerIntervalFwd = (forwardSpeed * (ctrlIntervalMS / 1000.0)) / cmPerEncoderTick;
  int16_t ticksPerIntervalTurn = (robotRadius * 3.14 / 180.0) * (turningSpeed * (ctrlIntervalMS / 1000.0)) / cmPerEncoderTick;

  leftMotor.setTargetSpeed(ticksPerIntervalFwd - ticksPerIntervalTurn);
  rightMotor.setTargetSpeed(ticksPerIntervalFwd + ticksPerIntervalTurn);
}

void Chassis::driveFor(float forwardDistance, float forwardSpeed, bool block) {
  // ensure the speed and distance are in the same direction
  forwardSpeed = forwardDistance > 0 ? fabs(forwardSpeed) : -fabs(forwardSpeed);
  setTwist(forwardSpeed, 0);  //sets the speeds

  // calculate the total motion in encoder ticks
  long delta = forwardDistance / cmPerEncoderTick;

  // set both wheels to move the same amount
  leftMotor.moveFor(delta);
  rightMotor.moveFor(delta);

  if (block) {
    while (!checkMotionComplete()) { delay(1); }
  }
}

void Chassis::driveWithTime(float forwardDistance, float targetSeconds) {
  unsigned long startTime = millis();
  float targetSeconds2 = targetSeconds - 0.02;
  float forwardSpeed = calculateSpeed(forwardDistance, targetSeconds2, 0);
  setTwist(forwardSpeed, 0);
  // calculate the total motion in encoder ticks
  long delta = forwardDistance / cmPerEncoderTick;
  // set both wheels to move the same amount
  leftMotor.moveFor(delta);
  rightMotor.moveFor(delta);

  while (true) {
    delay(1);
    if (checkMotionComplete()) break;
    float elapsedSeconds = (millis() - startTime) / 1000.0;
    forwardSpeed = calculateSpeed(forwardDistance, targetSeconds2, elapsedSeconds);
    int ticksPerIntervalFwd = (forwardSpeed * (ctrlIntervalMS / 1000.0)) / cmPerEncoderTick;
    leftMotor.targetSpeed = ticksPerIntervalFwd;
    rightMotor.targetSpeed = ticksPerIntervalFwd;
  }
  delay(20);
}

void Chassis::turnFor(float turnAngle, float turningSpeed, bool block) {
  // ensure angle and speed are in the same direction
  turningSpeed = turnAngle > 0 ? fabs(turningSpeed) : -fabs(turningSpeed);
  setTwist(0, turningSpeed);

  // calculate the total motion in encoder ticks
  long delta = turnAngle * (robotRadius * 3.14 / 180.0) / cmPerEncoderTick;

  // set wheels to drive in opposite directions
  leftMotor.moveFor(-delta);
  rightMotor.moveFor(delta);

  if (block) {
    while (!checkMotionComplete()) { delay(1); }
  }
}

void Chassis::turnWithTimePosPid(int targetCount, float targetSeconds) {
  unsigned long startTime = millis();
  targetSeconds = targetSeconds;
  leftMotor.setTargetCount(0);
  rightMotor.setTargetCount(0);
  while (true) {
    delay(1);
    float elapsedSeconds = (millis() - startTime) / 1000.0;
    int thisTarget = calculateIntermediateTargetLinear(targetCount, targetSeconds - 0.05, elapsedSeconds);
    leftMotor.targetCount = thisTarget;
    rightMotor.targetCount = -thisTarget;
    if (elapsedSeconds > targetSeconds)
      break;
  }
  delay(50);
  setMotorEfforts(0, 0);
}

bool Chassis::checkMotionComplete(void) {
  bool complete = leftMotor.checkComplete() && rightMotor.checkComplete();
  return complete;
}

/**
 * ISR for timing. On overflow of Timer4, the ISR takes a 'snapshot' of the encoder counts 
 * and then raises a flag to let the main program know it is time to execute the PID calculations.
 * 
 * Do not edit this function -- adding lengthy function calls will cause headaches.
 * */
ISR(TIMER4_OVF_vect) {
  chassis.updateEncoderDeltas();
}

void Chassis::updateEncoderDeltas(void) {
  leftMotor.calcEncoderDelta();
  rightMotor.calcEncoderDelta();

  leftMotor.update();
  rightMotor.update();
}

void Chassis::printSpeeds(void) {
  Serial.print(leftMotor.speed);
  Serial.print('\t');
  Serial.print(rightMotor.speed);
  Serial.print('\n');
}

void Chassis::printEncoderCounts(void) {
  Serial.print(leftMotor.getCount());
  Serial.print('\t');
  Serial.print(rightMotor.getCount());
  Serial.print('\n');
}
