/*
 * Testing function for the motor controller
 */


#include "Nidec24H.h"
#include "Encoder.h"
#include "PID.h"
#include <SimpleKalmanFilter.h>  // https://github.com/denyssene/SimpleKalmanFilter
#include <stdint.h>

// Define the pins
#define LED_PIN A0

#define MTR_PWM_PIN 9
#define MTR_BRAKE_PIN 7
#define MTR_DIR_PIN 8

#define ENC_A_PIN 2
#define ENC_B_PIN 3

// Time last update received from the controller
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
  //Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

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
  //testControllerFunctions();
  //testKnobPower();
  testChangingSpeed();
  //testKnobControl();
  //testChangingPower();
  //testFilter();
  //testDrivingSpeed();
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

//=====TEST FUNCTIONS================================================
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
 * Test changing the speed to a new value every three seconds.
 */
void testChangingPower() {
  const int speeds[] = {0,500,500,500};
  const int numSpeeds = 4;
  static int speedIndex = 0;
  static unsigned long lastSpeedChange = millis();
  static unsigned long lastPrint = millis();
  static unsigned long lastUpdate = micros();
  
  static int power = 0; //power level

  //go to the next speed if past time
  if (millis() - lastSpeedChange > 1000) {
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
    pid.calculatePID(encoder.estimateSpeed());
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
    power = pid.calculatePID(encoder.estimateSpeed());
    lastUpdate = micros();
  }

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(0); //blue
    Serial.print(",\t");
    Serial.print(pid.getTarget()); //red
    Serial.print(",\t");
    Serial.println(encoder.getFilteredSpeed()); //green
    
    //set the target speed
    int speed = knobFilter.filter((analogRead(KNOB_PIN)-500) * 8);
    if (abs(speed) < 50) speed = 0;
    pid.setTarget(speed);
    
    lastPrint = millis();
  }
}


/**
 * Test changing the speed to a new value every three seconds.
 */
void testChangingSpeed() {
  const int speeds[] = {0,500,800,200};
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
      case 'n':
        N = val;
      break;
      
    }
    pid.setPIDConsts(kp, ki, kd, N, UPDATE_INTERVAL);
    delay(500);
  }

  //go to the next speed if past time
  if (millis() - lastSpeedChange > 2000) {
    speedIndex++;
    lastSpeedChange = millis();
    if (speedIndex >= numSpeeds) {
      speedIndex = 0;
    }
  }

  //set the target speed
  pid.setTarget(speeds[speedIndex]);

  //Get the power level from the speed controller
  if (micros() - lastUpdate > UPDATE_INTERVAL) {
    power = pid.calculatePID(encoder.estimateSpeed());
    lastUpdate = micros();
  }

  //set the motor power
  motor.setPower(power);
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print("0,\t");
    Serial.print(pid.Setpoint);
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
          pid.reset();
          Serial.print(", reset controller");
        }
        pid.setTarget(val);
        motorState = SPEED_CONTROL;
        Serial.print(", new state: ");
        Serial.print(motorState);
        Serial.print(", Set speed: ");
        Serial.println(pid.getTarget());
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
          pid.reset();
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
      power = pid.calculatePID(encoder.getFilteredSpeed());
  
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
 * Test changing the speed to simulate driving the robot.
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
    pid.setTarget(driveSpeed);
    
    //set the motor power
    power = pid.calculatePID(encoder.estimateSpeed());
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
