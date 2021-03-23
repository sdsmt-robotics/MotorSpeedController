/* 
 * Class for Encoder speed calculator.
 * This class provides functions for tracking encoder ticks and determining speed.
 * 
 * init() - initialize the motor class.
 * setPower(int power) - set the power level for the motor
 * brake() - set the motor to brake.
 * setDirection(Direction invertDir) - set the default the motor direction.
 * getDirection() - get the motor direction
 * 
 * Note: this class is only set up to support four encoders. More is possible, 
 * but they will have to be added.
 */

#include "Encoder.h"

// for use by ISR routine
Encoder * Encoder::instance;
/**
 * @brief Constructor for the class.
 * 
 * @param aPin - First encoder pin. Attached to interrupt.
 * @param bPin - second encoder pin.
 * @param ticksPerRotation - Number of encoder ticks per rotation.
 */
Encoder::Encoder(int aPin, int bPin, int ticksPerRotation) 
    : aPin(aPin), bPin(bPin), ticksPerRotation(ticksPerRotation), speedFilter(300, 300, 0.025)  {
//      : encoder(dioPin, csPin, clkPin), speedFilter(150, 150, 0.05)  {
}

/**
 * @brief Initialize the motor.
 */
void Encoder::init() {
    //Setup control pins
    pinMode(aPin, INPUT_PULLUP);
    pinMode(bPin, INPUT_PULLUP);
    
    //Set up the encoder interrupt pin
    int intNumA = digitalPinToInterrupt(aPin);
    int intNumB = digitalPinToInterrupt(bPin);
    instance = this;
    attachInterrupt(intNumA, Encoder::isrA, CHANGE);
    attachInterrupt(intNumB, Encoder::isrB, CHANGE);

    // Get the register and bit for the pins to make the ISR a bit faster
    aPinRegister = portInputRegister(digitalPinToPort(aPin));
    aPinBit = digitalPinToBitMask(aPin);
    bPinRegister = portInputRegister(digitalPinToPort(bPin));
    bPinBit = digitalPinToBitMask(bPin);
    
  /*  encoder.init();
    //initialize the timer
    lastTickTime = micros();
    lastEstTime = lastTickTime;
    encoder.resetCounter();*/
}


/**
 * @brief set whether the encodeer's readings should be inverted.
 * 
 * @param invertDir - true if should be inverted, false otherwise
 */
void Encoder::invertDirection(bool invertDir) {
  this->invertDir = invertDir;
}

/**
 * Update the current speed estimate and return the filtered value.
 */
int Encoder::estimateSpeed() {
  // Calculate the speed if we got a tick. Otherwise assume 0.
  if (lastTickTime != lastEstTime) {
    speed = ticks * (tickConversion / (lastTickTime - lastEstTime));
  } else {
    speed = 0;
  }
  
  // Reset things
  lastEstTime = lastTickTime;
  ticks = 0;

  // Invert if needed
  if (invertDir) {
    speed = -speed;
  }

  // Filter
  filteredSpeed = speedFilter.updateEstimate(speed);


  // Return the speed
  return filteredSpeed;/*
  // Get current values
  unsigned long curTime = micros();
  long curAngle = encoder.readMultiTurnAngle();

  // Calculate the speed
  speed = long(curAngle - lastAngle) * tickConversion / (curTime - lastEstTime);
  filteredSpeed = speedFilter.updateEstimate(speed);

  // Reset for next time
  encoder.resetCounter();
  lastAngle = encoder.readMultiTurnAngle();
  lastEstTime = micros();

  return filteredSpeed;*/
}

/**
 * @brief Get the current speed the motor is running at in RPMs.
 * 
 * @return the current speed of the motor.
 */
int Encoder::getSpeed() {
  return speed;
}

/**
 * @brief Get the filtered current speed the motor is running at in RPMs.
 * 
 * @return the filtered current speed of the motor.
 */
int Encoder::getFilteredSpeed() {
  return filteredSpeed;
}

/**
 * Count a tick and set the last tick time.
 * 
 * NOTE: This ISR takes about 8 microseconds.
 * 
 * @param trigA - interrupt triggered by channel A or no.
 */
void Encoder::tick(bool trigA) {
  lastTickTime = micros();

  // Get current pin values
  bool aLevel = (*aPinRegister) & aPinBit;
  bool bLevel = (*bPinRegister) & bPinBit;

  // Increase or decrease depending on combination and which pin triggered the interrupt
  if (trigA != (aLevel == bLevel)) {
    ++ticks;
  } else {
    --ticks;
  }
}


/**
 * @brief Handle interrupt. Forward it to stored instance.
 */
void Encoder::isrA() {
  instance->tick(true);
}

void Encoder::isrB() {
  instance->tick(false);
}
