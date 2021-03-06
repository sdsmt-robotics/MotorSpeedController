/**
 * Class for interfacing with the Nidec24h motor controller.
 * 
 * init() - intialize the class.
 * 
 * setSpeed(speed) - Set the speed for the motor in rpm.
 * setPower(power) - Set the power for the motor in the range -1000 to 1000.
 * brake() - invertDirection(invertDir) - set whether motor control should be inverted (t/f).
 * 
 * getSpeed() - Get the current speed of the motor in rpm.
 * getPower() - Get the current set power of the motor in the range -1000 to 1000.
 * getTargetSpeed() - Get the set target speed of the controller.
 */

#include "Nidec24hController.h"

// Commands
#define SET_SPEED        0x01
#define SET_POWER        0x02
#define SET_BRAKE        0x03
#define SET_INVERT_DIR   0x04

#define GETTER_COMAND    0x10
#define GET_SPEED        0x00 | GETTER_COMAND
#define GET_POWER        0x01 | GETTER_COMAND
#define GET_TARGET_SPEED 0x02 | GETTER_COMAND


/**
 * @brief Constructor for the class.
 */
Nidec24hController::Nidec24hController(SPIClass &spi, int ssPin) : spi(spi), ssPin(ssPin) {
  
}

/**
 * Initialize the class.
 */
void Nidec24hController::init() {
  digitalWrite(SS, HIGH);  // ensure SS stays high for now
  digitalWrite(ssPin, HIGH);

  // Initialize motor control
  invertDirection(0);
  setPower(0);
}


/**
 * @brief Set the target speed of the motor in RPM.
 * 
 * @param speed - new speed for the motor.
 */
void Nidec24hController::setSpeed(int speed) {
  sendVal(SET_SPEED, speed);
}

/**
 * @brief Set the power for the motor. Value in the range -1000 to 1000.
 * 
 * @param power - new power level for the motor
 */
void Nidec24hController::setPower(int power) {
  sendVal(SET_POWER, power);
}

/**
 * @brief Stop the motor and engage braking.
 */
void Nidec24hController::brake() {
  sendVal(SET_BRAKE, 0);
}

/**
 * @brief Set whether motor direction should be inverted.
 * 
 * @param invertDir - set whether direction should be inverted
 */
void Nidec24hController::invertDirection(bool invertDir) {
  sendVal(SET_INVERT_DIR, invertDir);
}

/**
 * @brief Get the current speed the motor is running at in rpm.
 * 
 * @return the speed of the motor
 */
int Nidec24hController::getSpeed() {
  return getVal(GET_SPEED);
}

/**
 * @brief Get the current power set for the motor. In the range -1000 to 1000.
 * 
 * @return the power level of the motor.
 */
int Nidec24hController::getPower() {
  return getVal(GET_POWER);
}

/**
 * @brief Get the target speed (in rpm) the motor is trying to run at.
 * 
 * @return the target speed.
 */
int Nidec24hController::getTargetSpeed() {
  return getVal(GET_TARGET_SPEED);
}

/**
 * @brief Send a value to the device for the given command.
 * 
 * @param command - action to perform.
 * @param value - corresponding value for the action.
 */
void Nidec24hController::sendVal(int command, int value) {
  communicate(command, value);
}

/**
 * @brief Get a value from the device for the given command.
 * 
 * @param command - tell device which value to get.
 * @return the value returned from the device.
 */
int Nidec24hController::getVal(int command) {
  return communicate(command, 0);
}

/**
 * Communicate with the device. Send a command and then send a 16-bit value for the 
 * command and simultaneously read a response if present.
 * 
 * @param command - action to perform or value to get.
 * @param sendVal - corresponding value for the action.
 * @return the value returned from the device.
 */
int Nidec24hController::communicate(int command, int sendVal) {
  byte readVal[2];
  static unsigned long startTime;

  // Select the chip to control
  digitalWrite(ssPin, LOW);
  
  // Send the initial command
  spi.transfer(command);
  delayMicroseconds(TRANSFER_BYTE_DELAY);

  // Send the value and (or) read the response
  readVal[0] = spi.transfer(sendVal & 0xFF);
  delayMicroseconds(TRANSFER_BYTE_DELAY);
  readVal[1] = spi.transfer(sendVal >> 8);
  
  // Done controlling
  digitalWrite(ssPin, HIGH);
  delayMicroseconds (TRANSFER_BYTE_DELAY);

  // Turn the two values into a 16-bit value
  return (readVal[1] << 8) | readVal[0];
}
