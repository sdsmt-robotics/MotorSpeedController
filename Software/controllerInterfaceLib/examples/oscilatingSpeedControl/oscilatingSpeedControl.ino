/**
 * Demo motor control setting speeds rather quickly to obtain an oscilating speed.
 * 
 * Wiring:
 * SPI connections (MOSI, MISO, SCK, and SS). Can use a different pin for SS
 * if doing multiple motors.
 */


#include <SPI.h>
#include "Nidec24hController.h"

// Motor Controller class
Nidec24hController motor(SPI, SS);

// Time between settings speeds for motor controller
const unsigned long UPDATE_INTERVAL = 2500;


//===SETUP=============================
void setup (void) {
  Serial.begin (115200);

  // Initialize the SPI communications
  SPI.begin ();
  SPI.setClockDivider(SPI_CLOCK_DIV4);

  // Initialize the motor control
  motor.init();
}

//===LOOP=============================
void loop (void) {
  // How the robot is driving
  const int rotation = 400;   // chassis rotation (rpm)
  const int translation = 2;  // Driving feet per sec

  // Calculate the components of motor rotation for the translating and chassis rotation
  const float roatateForSpin = rotation * 4.0;  // Componenent of motor speed for chassis rotation
  const float frequency = rotation * 2 * M_PI / 60000000.0;
  const int rotateForTranslate = translation * 97.021;  //amount of rotation needed to get the translation speed
  
  static int driveSpeed = 0;  // Current set speed
  static unsigned long lastPrint = millis();
  static unsigned long lastUpdate = micros();

  
  // Get the power level from the speed controller
  if (micros() - lastUpdate > UPDATE_INTERVAL) {
    // Calculate the speed
    driveSpeed = roatateForSpin + rotateForTranslate * cos(micros() * frequency);
    motor.setSpeed(driveSpeed);

    lastUpdate = micros();
  }

  
  //Output current speed
  if (millis() - lastPrint > 10) {
    Serial.print(driveSpeed);
    Serial.print(",\t");
    Serial.print(motor.getSpeed());
    Serial.print(",\t");
    Serial.println(0);
    
    lastPrint = millis();
  }
}  // end of loop
