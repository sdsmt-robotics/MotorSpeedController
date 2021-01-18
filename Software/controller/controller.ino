/*
 * Testing program for the Nidec brushless motor speed controller class.
 * This is set up to run with: 
 *   - a Nidec brushless motor 
 *   - CUI encoder set to 48 ticks per rotation
 *   - Arduino Uno.
 * 
 * This will also run on an ATtiny.
 * 
 */

//Potential issues:
// - Incoming tick interrupts changing values while other processes are doing calculations

#include "Nidec24H.h"
#include "Encoder.h"
#include "Filter.h"
#include "SpeedController.h"
#include "AS5134.h"
#include <SimpleKalmanFilter.h>

#define TESTING 1

// Define the pins
#define LED_PIN 1

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
const byte SPI_COM_END = 2;          // Number of the final send/receive byte
volatile int spiVal = 0;                     // Value being sent or received
volatile unsigned long lastByteTime = 0;  // Last communication time
volatile bool dataReceived = false;

// Byte transmission timeout in microseconds
const unsigned long COM_TIMEOUT = 500;

//Create the motor control object
//Nidec24H(dirPin, brakePin)
Nidec24H motor(/*MTR_PWM_PIN, */MTR_DIR_PIN, MTR_BRAKE_PIN);

// Create the encoder reader object
// Encoder(int aPin, int bPin, int ticksPerRotation)
Encoder encoder(ENC_A_PIN, ENC_B_PIN, 360);
AS5134 as5134(4, 5, 6);
//Encoder encoder(2, 3, 4);

//motor speed controller
float kp = 0.3, ki = 10.0, kd = 0.009;
SpeedController speedControl;

// Current speed of the motor
int curSpeed = 0; 

// Motor control states
enum MotorState {SPEED_CONTROL, POWER_CONTROL, BRAKE};
MotorState motorState = POWER_CONTROL;

// Time last update received from the controller
// TODO: add a timeout?
unsigned long lastUpdate = 0;

// Controller update interval in microseconds
const unsigned long UPDATE_INTERVAL = 2500;


//=====SETUP=========================================================
void setup() {
  Serial.begin(115200);
  //pinMode(LED_PIN, OUTPUT);
  
  // Initialize the SPI for the device
  initSpi();

  //initialize the speed control
  //kp - amplitude
  //ki - constant error correction
  //kd - damper
  // TODO: add support for reading and saving these values from EEPROM
  //setPIDConsts(kp, ki, kd)
  //speedControl.setPIDConsts(0.08, 1.4, 0.003);  //RS550
  //speedControl.setPIDConsts(1.6, 10.0, 0.03); //Was working at one point
  speedControl.setPIDConsts(kp, ki, kd);
  speedControl.setOutputLimits(-1000, 1000);
  speedControl.setTarget(0);

  
  // initialize the motor
  motor.init();

  // Initialize the encoder 
  encoder.init();

  Serial.println("Start");
}


//=====LOOP=========================================================
unsigned long delta[] = {0, 0, 0};
bool newInt = false;
void loop() {
  static unsigned long lastUpdate = micros();  // Last time the current speed and motor control PID got updated

  // Handle received data if we got some
  if (dataReceived) {
    dataReceived = false;
    handleReceivedData(spiCommand, spiVal);
  }
  
  // Control the motor
  if (micros() - lastUpdate > UPDATE_INTERVAL) {
    encoder.estimateSpeed();
    
    if (motorState == SPEED_CONTROL) {
      //Get the power level from the speed controller
      int power = speedControl.calcOutput(encoder.getFilteredSpeed());
  
      //set the motor power
      motor.setPower(power);
    }
    
    lastUpdate = micros();
  }
  //testControllerFunctions();
  //testKnobPower();
  //testChangingSpeed();
  //testKnobControl();
  //testChangingPower();
  //testFilter();

  /*byte curspiByteNum = spiByteNum;
  if ((micros() - lastByteTime > COM_TIMEOUT) && (spiByteNum != 0)) {  //(have to check val before and after since dealing with timer vals takes a while)
    Serial.print("Reset: ");
    Serial.println(curspiByteNum);
    //spiByteNum = 0;
  }*/

  
  /*if (newInt) {
    Serial.print(delta[0]);
    Serial.print("\t");
    Serial.print(delta[1]);
    Serial.print("\t");
    Serial.println(delta[2]);
    newInt = false;
  }*/
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

  // Initialize SPI timeout interrupt
  // Set for 2.5kHz, compare match mode, on timer 2.
  TIMSK2 = 0; // Diable interrupts
  TCCR2A = 0;
  TCCR2B = 0;  // 
  TCNT2  = 0;  // Clear counter

  OCR2A = (F_CPU/128)/2500;              // compare match register
  TCCR2A |= (1 << WGM21);                // CTC mode
  TCCR2B |= (1 << CS20) | (1 << CS22);   // 128 prescaler 

  Serial.print("Timeout: ");
  Serial.print(OCR2A);
  Serial.print(", ");
  Serial.println(F_CPU);
}


/**
 * @brief Construct the ISR for the SPI communications. Called when SPDR gets a new byte.
 */
 unsigned long runTimer = 0;
ISR (SPI_STC_vect) {
  // Handle based on which byte this is
  if (spiByteNum == 0) {  // First byte
    startTimeout();
    
    // Read the fist byte (the command)
    spiCommand = SPDR;
    
    // Get the value if doing a send
    if (spiCommand & GETTER_COMAND) {
      switch (spiCommand) {
        case GET_SPEED:  // Current speed of the motor
          spiVal = encoder.getFilteredSpeed();
          break;

        case GET_POWER:  // Current power set for the motor
          spiVal = motor.getPower();
          break;

        case GET_TARGET_SPEED:  // Target speed of the motor
          spiVal = speedControl.getTarget();
          break;
        
        default: // Just send 0. Garbage request
          spiVal = 0x0;
          break;
      }
      
      //First byte
      SPDR = spiVal & 0xFF;
    }
    ++spiByteNum;
  } else if (spiByteNum >= SPI_COM_END) {  // Last byte
    clearTimeout();
    spiByteNum = 0;

    // Get the last byte if receiving data
    if (!(spiCommand & GETTER_COMAND)) {
      dataReceived = true;
      spiVal |= SPDR << 8;
    }
  } else {
    // Send or receive the byte
    if (spiCommand & GETTER_COMAND) { // Sending
      SPDR = spiVal >> 8;
    } else { // Receiving
      spiVal = SPDR;
    }
    
    ++spiByteNum;
  }
}

void handleReceivedData(byte command, int value) {
    // Do the thing with the value if receiving
    if (!(command & GETTER_COMAND)) {
      // Do what we need to with it
      switch (command) {
        case SET_SPEED:  // Set current speed of the motor
          if (motorState != SPEED_CONTROL) { // Do a reset if changing modes so don't jump.
            speedControl.reset();
          }
          speedControl.setTarget(value);
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
            speedControl.reset();
          }
        break;
      }
    }
}

/**
 * Enable the timeout countdown timer for restarting communications after transmission start.
 */
inline void startTimeout() {
  TCNT2  = 0;  // Clear counter
  TIMSK2 &= ~(1 << OCIE2A);  // Disable the interrupt
}

inline void clearTimeout() {
  TIMSK2 = 0;
  TIFR2 |= (1 << OCF2A);  // Clear the interrupt bit if set (cleared by writting a one)
  TCNT2  = 0;
}

// Receive timeout interrupt.
ISR(TIMER2_COMPA_vect) {
  // Disable interrupt
  clearTimeout();

  // Do the reset
  spiByteNum = 0;
  Serial.println('-');
}

//=====TEST FUNCTIONS================================================
#if TESTING
/**
 * Test basic motor functionality.
 */
void testMotorFunctions() {
    Serial.println("Running...");
    motor.setPower(30);
    delay(3000);

    Serial.println("Braking...");
    motor.brake();
    delay(3000);

    Serial.println("Running...");
    motor.setPower(80);
    delay(3000);

    Serial.println("Stopping...");
    motor.setPower(0);
    delay(3000);
}

void tickCount() {
  static long lastPrint = millis();
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.println(encoder.tickCount);
    
    
    lastPrint = millis();
  }
}


/**
 * Control motor speed target using a knob on A5.
 */
void testKnobPower() {
  const int KNOB_PIN = A1;
  static int power = 0; //power level
  static long lastPrint = millis();
  static Filter knobFilter(10);
  static unsigned long lastUpdate = millis();

  if (millis() - lastUpdate > 20) {
    //set the motor power
    power = knobFilter.filter(analogRead(KNOB_PIN));
    motor.setPower(power);
  }
  
  //Output current values
  if (millis() - lastPrint > 20) {
    Serial.println(power);
    Serial.print(",\t");
    Serial.println(encoder.estimateSpeed());
    
    lastPrint = millis();
  }
}

/**
 * Run the motor at some constant power level and read the tick frequency.
 */
void testTickFreq() {
  const int power = 100; //power level
  static long lastPrint = millis();

  //Do the calculation just so we get the same amount of lag as normal
  //speedControl.calcOutput(encoder.getFilteredSpeed());

  //set the motor power
  motor.setPower(power);

  //motor.ticks = 0;
  delay(100);
//  Serial.println(motor.ticks);
}


/**
 * Test changing the speed to a new value every three seconds.
 */
void testChangingPower() {
  const int speeds[] = {500, 120, 800, 300};
  const int numSpeeds = 4;
  static int speedIndex = 0;
  static unsigned long lastSpeedChange = millis();
  static unsigned long lastPrint = millis();
  static unsigned long lastUpdate = micros();
  
  static int power = 0; //power level

  //go to the next speed if past time
  if (millis() - lastSpeedChange > 3000) {
    speedIndex++;
    lastSpeedChange = millis();
    if (speedIndex >= numSpeeds) {
      speedIndex = 0;
    }
  }

  //set the motor power
  power = speeds[speedIndex];
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(power);
    Serial.print(",\t");
    Serial.print(encoder.estimateSpeed());
    Serial.print(",\t");
    Serial.println(encoder.getSpeed());
    
    lastPrint = millis();
  }
}

/**
 * Run the motor at some constant power level and observe the raw speed vs the filtered speed.
 */
void testFilter() {
  const int power = 200; //power level
  static unsigned long lastPrint = millis();
  static unsigned long lastUpdate = micros();
  //static int as5134Speed = 0;

  //Get the power level from the speed controller
  if (micros() - lastUpdate > UPDATE_INTERVAL) {
    speedControl.calcOutput(encoder.estimateSpeed());
    //as5134Speed = getAs5134Speed();
    lastUpdate = micros();
  }

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(encoder.getSpeed());
    Serial.print(",\t");
    Serial.println(encoder.getFilteredSpeed());
    //Serial.print(",\t");
    //Serial.println(as5134Speed);
    
    lastPrint = millis();
  }
}


int getAs5134Speed() {
  static unsigned long curTime, lastTime = micros();
  static int curPos, lastPos = as5134.readMultiTurnAngle();
  static int curSpeed;
  static SimpleKalmanFilter speedFilter(150, 150, 0.05);

  //Calculate the speed based on the current position
  curPos = as5134.readMultiTurnAngle();
  curTime = micros();
  curSpeed = long(curPos - lastPos) * 166667 / (curTime - lastTime); //rpm
  //curSpeed = (curPos - lastPos); //rpm
  
  as5134.resetCounter();
  lastPos = as5134.readMultiTurnAngle();
  lastTime = micros();
  
  return speedFilter.updateEstimate(curSpeed);
}

/**
 * Run the motor at a set speed using the PID control.
 */
void testSetSpeed() {
  static int power = 0; //power level
  static long lastPrint = millis();

  //set the target speed
  speedControl.setTarget(400);

  //Get the power level from the speed controller
  power = speedControl.calcOutput(encoder.getFilteredSpeed());

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(0);
    Serial.print(",\t");
    Serial.print(speedControl.getTarget());
    Serial.print(",\t");
    Serial.println(encoder.getFilteredSpeed());
    
    lastPrint = millis();
  }
}


/**
 * Control motor speed target using a knob on A5.
 */
void testKnobControl() {
  const int KNOB_PIN = A1;
  static Filter knobFilter(10);
  static int power = 0; //power level
  static unsigned long lastPrint = millis();
  static unsigned long lastUpdate = micros();


  //Get the power level from the speed controller
  if (micros() - lastUpdate > UPDATE_INTERVAL) {
    power = speedControl.calcOutput(encoder.estimateSpeed());
    lastUpdate = micros();
  }

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(0); //blue
    Serial.print(",\t");
    Serial.print(speedControl.getTarget()); //red
    Serial.print(",\t");
    Serial.print(speedControl.lastError); //green
    Serial.print(",\t");
    Serial.print(speedControl.integral); //orange
    Serial.print(",\t");
    Serial.print(speedControl.derivative); //purple
    Serial.print(",\t");
    Serial.print(encoder.getFilteredSpeed()); //grey
    Serial.print(",\t");
    Serial.println(speedControl.kp * speedControl.lastError + speedControl.ki * speedControl.integral + speedControl.kd * speedControl.derivative); //blue
    
    
    //set the target speed
    int speed = knobFilter.filter((analogRead(KNOB_PIN)-500) * 8);
    if (abs(speed) < 50) speed = 0;
    speedControl.setTarget(speed);
    
    lastPrint = millis();
  }
}

/**
 * Run the motor at a set speed using the PID control.
 */
void testOscSpeed() {
  const int rotation = 30;
  const float c = rotation * 2 * M_PI / 60000000.0;
  
  static int power = 0; //power level
  
  static long lastPrint = millis();
  
  int driveSpeed = 1000 + 1000 * cos(micros() * c);

  //set the target speed
  speedControl.setTarget(driveSpeed);

  //Get the power level from the speed controller
  power = speedControl.calcOutput(encoder.getFilteredSpeed());

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(speedControl.getTarget());
    Serial.print(",\t");
    Serial.println(encoder.getFilteredSpeed());
    
    lastPrint = millis();
  }
  
    //motor.ticks = 0;
}

/**
 * Test changing the speed to a new value every three seconds.
 */
void testChangingSpeed() {
  const int speeds[] = {900, 500, 1600, 400};
  const int numSpeeds = 4;
  static int speedIndex = 0;
  static unsigned long lastSpeedChange = millis();
  static unsigned long lastPrint = millis();
  static unsigned long lastUpdate = micros();
  
  static int power = 0; //power level

  // Adjust PID vals
  // Format: Xd.dddd where X is the thing to change and d.dddd is the new value.
  if (Serial.available() > 0) {
    char change = Serial.read();
    Serial.setTimeout(10);
    float val = Serial.parseFloat();

    switch (change) {
      case 'p':
        kp = val;
      break;
      case 'i':
        ki = val;
      break;
      case 'd':
        kd = val;
      break;
      
    }
    speedControl.setPIDConsts(kp, ki, kd);
    delay(1000);
  }

  //go to the next speed if past time
  if (millis() - lastSpeedChange > 3000) {
    speedIndex++;
    lastSpeedChange = millis();
    if (speedIndex >= numSpeeds) {
      speedIndex = 0;
    }
  }

  //set the target speed
  speedControl.setTarget(speeds[speedIndex]);

  //Get the power level from the speed controller
  if (micros() - lastUpdate > UPDATE_INTERVAL) {
    power = speedControl.calcOutput(encoder.estimateSpeed());
    lastUpdate = micros();
  }

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(speedControl.getTarget());
    Serial.print(",\t");
    Serial.print(power);
    Serial.print(",\t");
    Serial.println(encoder.getFilteredSpeed());
    
    lastPrint = millis();
  }
}


/**
 * Test controlling basic functions with keyboard input.
 */
void testControllerFunctions() {
  static int power = 0; //power level
  static unsigned long lastUpdate = micros();
  
  if (Serial.available() > 0) {
    char command = Serial.read();
    Serial.setTimeout(10);
    int val = Serial.parseInt();

    switch (command) {
      case 's':  // Set current speed of the motor
        Serial.print("Old state: ");
        Serial.print(motorState);
        if (motorState != SPEED_CONTROL) { // Do a reset if changing modes so don't jump.
          speedControl.reset();
          Serial.print(", reset controller");
        }
        speedControl.setTarget(val);
        motorState = SPEED_CONTROL;
        Serial.print(", new state: ");
        Serial.print(motorState);
        Serial.print(", Set speed: ");
        Serial.println(speedControl.getTarget());
        break;

      case 'p':  // Set current speed for motor and go into power control mode
        Serial.print("Old state: ");
        Serial.print(motorState);
        motorState = POWER_CONTROL;
        motor.setPower(val);
        Serial.print(", new state: ");
        Serial.print(motorState);
        Serial.print(", Set power: ");
        Serial.println(motor.getPower());
        break;

      case 'b':  // Stop the motor
        motorState = BRAKE;
        motor.brake();
        Serial.println("Set brake.");
        break;
      
      case 'i':  // Set whether default motor direction should be inverted
        motor.invertDirection(val);
        encoder.invertDirection(val);
        if (motorState == SPEED_CONTROL) { // Do a reset if in speed control mode so don't jump.
          speedControl.reset();
        }
        Serial.print("Direction invert: ");
        Serial.println(val == true);
        break;
      
      default:
        Serial.println("Invalid command.");
        break;
    }
  }

  // Control the motor
  if (micros() - lastUpdate > UPDATE_INTERVAL) {
    encoder.estimateSpeed();
    
    if (motorState == SPEED_CONTROL) {
      //Get the power level from the speed controller
      power = speedControl.calcOutput(encoder.getFilteredSpeed());
  
      //set the motor power
      motor.setPower(power);
      
      //Serial.print(power);
      //Serial.print(",\t");
    }
    //Serial.println(motorState);
    
    lastUpdate = micros();
  }
}


/**
 * Test changing the speed to a new value every three seconds and display PID values.
 */
void testShowPID() {
  const int speeds[] = {1000, 600, 2000};
  const int numSpeeds = 3;
  static int speedIndex = 0;
  static unsigned long lastSpeedChange = millis();
  static unsigned long lastPrint = millis();
  static unsigned long lastUpdate = micros();
  
  static int power = 0; //power level
  

  //go to the next speed if past time
  if (millis() - lastSpeedChange > 3000) {
    speedIndex++;
    lastSpeedChange = millis();
    if (speedIndex >= numSpeeds) {
      speedIndex = 0;
    }
  }

  //set the target speed
  speedControl.setTarget(speeds[speedIndex]);

  //Get the power level from the speed controller
  if (micros() - lastUpdate > UPDATE_INTERVAL) {
    power = speedControl.calcOutput(encoder.estimateSpeed());
  }

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(speedControl.lastError);
    Serial.print(",\t");
    Serial.print(speedControl.integral);
    Serial.print(",\t");
    Serial.println(speedControl.derivative);
    
    lastPrint = millis();
  }
}

/**
 * Test changing the speed to a new value every three seconds.
 */
void testDrivingSpeed() {
  const int numSpeeds = 3;
  static int speedIndex = 0;
  static long lastSpeedChange = millis();
  const int rotation = 400; //rpm
  const int translation = 2;  //feet per sec
  const float roatateForSpin = rotation * 4.0;  // Componenent of motor speed for chassis rotation
  
  const float c = rotation * 2 * M_PI / 60000000.0;
  const int rotateForTranslate = translation * 97.021;  //amount of rotation needed to get the translation speed
  
  static int power = 0;       // Power level
  static int driveSpeed = 0;  // Current set speed
  static unsigned long lastPrint = millis();
  static unsigned long lastUpdate = micros();

  
  // Get the power level from the speed controller
  if (micros() - lastUpdate > UPDATE_INTERVAL) {
    // Calculate the speed
    driveSpeed = roatateForSpin + rotateForTranslate * cos(micros() * c);
    speedControl.setTarget(driveSpeed);
    
    //set the motor power
    power = speedControl.calcOutput(encoder.estimateSpeed());
    motor.setPower(power);

    lastUpdate = micros();
  }

  
  //Output current speed
  if (millis() - lastPrint > 10) {
    Serial.print(driveSpeed);
    Serial.print(",\t");
    Serial.print(encoder.getFilteredSpeed());
    Serial.print(",\t");
    Serial.println(0);
    
    lastPrint = millis();
  }
  
    //motor.ticks = 0;
}

/**
 * Control motor using commands sent from the serial console.
 * 
 * Space: toggle motor on/off
 * number: set the motor to that speed
 */
void testSpeedControls() {
  static bool motorRunning = true;
  static int power = 0; //power level
  
  static long lastPrint = millis();

  
  //Read user input if available
  if (Serial.available()) {
    //If space, toggle motor
    if (Serial.peek() == ' ') {
      motorRunning = !motorRunning;
      while (Serial.available()) {
        Serial.read();
      }
    } else {
      //read the new speed value
      motorRunning = true;
      speedControl.setTarget(Serial.parseInt());
    }
  }

  
  //update the motor
  if (motorRunning) {
    power = speedControl.calcOutput(encoder.getFilteredSpeed());
    motor.setPower(power);
  } else {
    motor.setPower(0);
  }
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(speedControl.getTarget());
    Serial.print(",\t");
    Serial.println(encoder.getFilteredSpeed());
    
    lastPrint = millis();
  }
}
#endif