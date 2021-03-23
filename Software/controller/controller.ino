/*
 * Main program for the motor controller. Receive commands and values over SPI and 
 * manage motor as instructed. The hardware is as follows:
 *  - ATMEGA328P
 *  - SN74AHCT541N (level shifter)
 *  - Nidec24H brushless DC motor
 *  - AS5134 magnetic encoder (used in quadrature mode)
 * 
 * The motor controller can be in three states:
 *  - Speed control
 *  - Power control
 *  - Brakeing
 * The motor direction can also be inverted. The commands to perform these actions 
 * are sent over SPI along with the associated values. Requests for current values 
 * can also be sent over SPI.
 * 
 * Notes on the SPI:
 *  - works well at around 1MHz. Might work faster, but can be iffish.
 *  - Should leave at least 200us between commmunications.
 *  - (For some reason) input voltage to the ATMEGA328P for the SPI must be <5V. Higher 
 *    will cause failed transmissions.
 * 
 */


#include "Nidec24H.h"
#include "Encoder.h"
#include "Filter.h"
#include "PID.h"
#include <SimpleKalmanFilter.h>  // https://github.com/denyssene/SimpleKalmanFilter

// Define the pins
#define LED_PIN A0

#define MTR_PWM_PIN 9
#define MTR_BRAKE_PIN 7
#define MTR_DIR_PIN 8

#define ENC_A_PIN 2
#define ENC_B_PIN 3

// Define SPI Commands and commnumication stuff
// Commands
#define SET_SPEED        0x01
#define SET_POWER        0x02
#define SET_BRAKE        0x03
#define SET_INVERT_DIR   0x04

#define GETTER_COMAND    0x10
#define GET_SPEED        0x00 | GETTER_COMAND
#define GET_POWER        0x01 | GETTER_COMAND
#define GET_TARGET_SPEED 0x02 | GETTER_COMAND

// SPI Send/receive variables
volatile byte spiCommand = 0;        // Command received from the controller
volatile byte spiByteNum = 0;        // Current send/receive status
volatile int16_t spiVal = 0;         // Value being sent or received
volatile bool dataReceived = false;  // Track if we have new data
const byte SPI_COM_END = 2;          // Number of the final send/receive byte
const long SPI_TIMEOUT = 100;        // Max time a byte should take (in microseconds)

// Struct for receiving data
union SpiData
{
  int16_t val;
  struct
  {
    uint8_t lsb;
    uint8_t msb;
  };
};

// Time last update received from the controller
// TODO: add a timeout?
unsigned long lastUpdate = 0;

// Controller update interval in microseconds
const unsigned long UPDATE_INTERVAL = 2500;

// Motor control states
enum MotorState {SPEED_CONTROL, POWER_CONTROL, BRAKE};
MotorState motorState = POWER_CONTROL;

//Create the motor control object
//Nidec24H(dirPin, brakePin)
Nidec24H motor(MTR_DIR_PIN, MTR_BRAKE_PIN);

// Create the encoder reader object
// Encoder(int aPin, int bPin, int ticksPerRotation)
Encoder encoder(ENC_A_PIN, ENC_B_PIN, 360);

//motor speed controller
// TODO: add support for reading and saving these values from EEPROM
float kp = 5, ki = 7, kd = 0.5 , N = 10;
// PID(Kp, Ki, Kd, N, sample_time)
PID pid(kp, ki, kd, N, UPDATE_INTERVAL);
double setpoint = 1;


//=====SETUP=========================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize the SPI for the device
  initSpi();

  //initialize the speed control
  pid.setLimits(-1000, 1000);
  pid.setTarget(setpoint);

  // initialize the motor
  motor.init();

  // Initialize the encoder 
  encoder.init();

  // Turn on the LED
  ledOn();
}


//=====LOOP=========================================================
void loop() {
  static unsigned long lastUpdate = micros();  // Last time the current speed and motor control PID got updated

  while (true) {
    // Control the motor
    if (micros() - lastUpdate > UPDATE_INTERVAL) {
      encoder.estimateSpeed();
      
      if (motorState == SPEED_CONTROL) {
        //Get the power level from the speed controller
        int power = pid.calculatePID(encoder.getFilteredSpeed());
    
        //set the motor power
        motor.setPower(power);
      }
      
      lastUpdate = micros();
    }
  }
}

/**
 * @brief Turn on the LED.
 */
void ledOn() {
  // Turn on the LED
  digitalWrite(LED_PIN, HIGH);
}

/**
 * @brief Turn off the LED.
 */
void ledOff() {
  // Turn off the LED
  digitalWrite(LED_PIN, LOW);
}

//=====SPI functions==========================
/**
 * @brief Initialize the SPI communications for the slave device.
 */
void initSpi() {
  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // turn on interrupts
  SPCR |= _BV(SPIE);
}


/**
 * @brief Construct the ISR for the SPI communications. Called when SPDR gets a new byte.
 */
ISR (SPI_STC_vect) {
  // Read the fist byte (the command)
  noInterrupts();

  spiCommand = SPDR;
  // Get the value if doing a send (Switch takes ~12us worst case, ~3.5us avg)
  if (spiCommand & GETTER_COMAND) {
    switch (spiCommand) {
      case GET_SPEED:  // Current speed of the motor
        spiVal = encoder.getFilteredSpeed();
        break;

      case GET_POWER:  // Current power set for the motor
        spiVal = motor.getPower();
        break;

      case GET_TARGET_SPEED:  // Target speed of the motor
        spiVal = pid.getTarget();
        break;
      
      default: // Just send 0. Garbage request
        spiVal = 0x0;
        break;
    }
  } else {
    spiVal = 0x0;
  }

  // Do the data transfer
  bool success = transferData(spiVal);

  // Handle data we received
  if (success) 
    handleReceivedData(spiCommand, spiVal);
    
  interrupts();
}


/**
* @brief do stuff with the data that was received over SPI.
* 
* @param command - the command that came with the data
* @param value - received data
*/
void handleReceivedData(byte command, int value) {
    // Do the thing with the value if receiving
    if (!(command & GETTER_COMAND)) {
      // Do what we need to with it
      switch (command) {
        case SET_SPEED:  // Set current speed of the motor
          if (motorState != SPEED_CONTROL) { // Do a reset if changing modes so don't jump.
            pid.reset();
          }
          pid.setTarget(value);
          motorState = SPEED_CONTROL;
          break;

        case SET_POWER:  // Set current speed for motor and go into power control mode
          motorState = POWER_CONTROL;
          motor.setPower(value);
          break;

        case SET_BRAKE:  // Stop the motor
          motorState = BRAKE;
          motor.brake();
          break;
        
        case SET_INVERT_DIR:  // Set whether default motor direction should be inverted
          motor.invertDirection(value);
          encoder.invertDirection(value);
          if (motorState == SPEED_CONTROL) { // Do a reset if in speed control mode so don't jump.
            pid.reset();
          }
        break;
      }
    }
}

/**
 * @brief transfer the data through SPI
 * 
 * @param data - the value to send and variable to take received value
 * @return true if transmission success, false otherwise (timeout).
 */
bool transferData(volatile int16_t &data) {
    static unsigned long start = micros();
    static SpiData in, out;

    in.val = data;

    // Send/Receive the LSB
    SPDR = in.lsb;
    asm volatile("nop");
    start = micros();
    while (!spiByteReceived()) {
      if (micros() - start > (SPI_TIMEOUT << 1)) {
        return false;
      }
    }
    out.lsb = SPDR;
    
    // Send/Receive the MSB
    SPDR = in.msb;
    asm volatile("nop");
    start = micros();
    while (!spiByteReceived()) {
      if (micros() - start > SPI_TIMEOUT) {
        return false;
      }
    }
    out.msb = SPDR;
    data = out.val;
    return true;
}

/**
* @brief check if a SPI transmission has been received
* 
* @return true if received a byte over SPI
*/
inline bool spiByteReceived() {
  return SPSR & _BV(SPIF);
}
