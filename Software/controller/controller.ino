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
#include "PID.h"
#include <stdint.h>

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
#define SET_TIMEOUT      0x05

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

const int MAX_RPM = 6000;  // Maximum RPM we could ask the motor to run

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
unsigned long lastReceive = 0;
unsigned long timeout = 0;
volatile bool receivedCommand = false;

// Controller update interval in microseconds
const unsigned long UPDATE_INTERVAL = 2000;

// Define blink patterns
union BlinkPattern
{
  unsigned long time[2];
  struct
  {
    unsigned long offTime;
    unsigned long onTime;
  };
};
const BlinkPattern BRAKE_BLINK = {1000, 200};
const BlinkPattern SPEED_BLINK = {500, 500};
const BlinkPattern POWER_BLINK = {200, 1000};
const BlinkPattern TIMEOUT_BLINK = {200, 200};

// Motor control states
enum MotorState {SPEED_CONTROL, POWER_CONTROL, BRAKE, TIMEOUT};
volatile MotorState motorState = TIMEOUT;

//Create the motor control object
//Nidec24H(dirPin, brakePin)
Nidec24H motor(MTR_DIR_PIN, MTR_BRAKE_PIN);

// Create the encoder reader object
// Encoder(int aPin, int bPin, int ticksPerRotation)
Encoder encoder(ENC_A_PIN, ENC_B_PIN, 360);

//motor speed controller
// TODO: add support for reading and saving these values from EEPROM
//float kp = 5, ki = 7, kd = 0.5 , kf = 1/10.0, N = 10;
float kp = 5, ki = 12, kd = 0.2 , kf = 1/10.0, N = 10;
//float kp = 1.0, ki = 0.0, kd = 0.0 , kf = 0.16, N = 1;
// PID(Kp, Ki, Kd, N, sample_time)
PID pid(kp, ki, kd, kf, N, UPDATE_INTERVAL);
double setpoint = 1;


//=====SETUP=========================================================
void setup() {
  // Initialize the SPI for the device
  initSpi();

  //initialize the speed control
  pid.setLimits(-1000, 1000);
  pid.setTarget(setpoint);

  // initialize the motor
  motor.init();

  // Initialize the encoder 
  encoder.init();

  // Init LED control
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  lastReceive = millis();
}


//=====LOOP=========================================================
void loop() {
  static unsigned long lastUpdate = micros();  // Last time the current speed and motor control PID got updated

  while (true) {
    // Timeout if haven't received a transmission in a while
    if (receivedCommand) {
      lastReceive = millis();
      receivedCommand = false;
    } else if ((motorState != TIMEOUT) && (timeout != 0) && (millis() - lastReceive > timeout)) {
      motorState = TIMEOUT;
      motor.setPower(0);
    }

    // Control the motor
    if (motorState != TIMEOUT) {
      // Control the motor
      if (micros() - lastUpdate > UPDATE_INTERVAL) {
        encoder.estimateSpeed();
        
        if (motorState == SPEED_CONTROL) {
          //Get the power level from the speed controller
          int power = pid.calculateOutput(encoder.getFilteredSpeed());
      
          //set the motor power
          motor.setPower(power);
        }
        
        lastUpdate = micros();
      }
    }

    // Update the LED blink
    updateLed();
  }
}

/**
 * @brief Control the LED blink pattern.
 */
void updateLed() {
  static bool state = LOW;
  static unsigned long lastTransition = millis();
  bool newState = state;

  // Figure out what pattern should blink
  if (motorState == SPEED_CONTROL) {
    if (millis() - lastTransition > SPEED_BLINK.time[state]) {
      newState = !state;
    }
  } else if (motorState == POWER_CONTROL) {
    if (motor.getPower() == 0) {
      newState = HIGH;
    } else if (millis() - lastTransition > POWER_BLINK.time[state]) {
      newState = !state;
    }
  } else if (motorState == BRAKE) {
    if (millis() - lastTransition > BRAKE_BLINK.time[state]) {
      newState = !state;
    }
  } else if (motorState == TIMEOUT) {
    if (millis() - lastTransition > TIMEOUT_BLINK.time[state]) {
      newState = !state;
    }
  }

  // Update the LED state if needed
  if (state != newState) {
    digitalWrite(LED_PIN, newState);
    state = newState;
    lastTransition = millis();
  }
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
  

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // turn on interrupts
  SPCR |= _BV(SPIE);

  // Initialize SPI timeout timer
  TIMSK2 = 0; // Diable interrupts
  TCCR2A = 0;
  TCCR2B = 0;  // 
  TCNT2  = 0;  // Clear counter

  TCCR2A |= (1 << WGM21);                // CTC mode
  TCCR2B |= (1 << CS21) | (1 << CS20);   // 128 prescaler 
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
  
  // Let the loop know we got a communicaiton
  receivedCommand = true;
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

          // Consrain to make sure not trying to set to high
          if (value > MAX_RPM)
            value = MAX_RPM;
          else if (value < -MAX_RPM)
            value = -MAX_RPM;
          
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
        case SET_TIMEOUT:
          timeout = value;
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
    static SpiData in, out;

    in.val = data;

    // Send/Receive the LSB
    SPDR = in.lsb;
    asm volatile("nop");
    resetMicroTimer();
    while (!spiByteReceived()) {
      if (microTimer() > (SPI_TIMEOUT << 1)) {
        return false;
      }
    }
    out.lsb = SPDR;
    
    // Send/Receive the MSB
    SPDR = in.msb;
    asm volatile("nop");
    resetMicroTimer();
    while (!spiByteReceived()) {
      if (microTimer() > SPI_TIMEOUT) {
        return false;
      }
    }
    out.msb = SPDR;
    data = out.val;
    return true;
}

/**
 * Get the current time in us since reset.
 */
unsigned microTimer() {
  // 16 MHz cpu, 32 prescaler
  return TCNT2 * 2;
}

/**
 * Reset the timer.
 */
void resetMicroTimer() {
  TCNT2 = 0;
}


/**
* @brief check if a SPI transmission has been received
* 
* @return true if received a byte over SPI
*/
inline bool spiByteReceived() {
  return SPSR & _BV(SPIF);
}
