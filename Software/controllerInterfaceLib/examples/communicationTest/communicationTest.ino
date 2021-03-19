/**
 * Iterate through increasing power values, write to motor controller, read back, and output error %.
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
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);

  // Initialize the motor control
  motor.init();

  Serial.println("Begin Oscilating Control");
}

//===LOOP=============================
void loop (void) {
  static int sendVal = 0;
  int receiveVal;
  static unsigned long lastUpdate = micros();
  static unsigned long numSent = 0;
  static unsigned long numDelayErrors = 0;
  static unsigned long numOthErrors = 0;


  // Get the power level from the speed controller
  if (micros() - lastUpdate > UPDATE_INTERVAL) {
    sendVal++;
    if (sendVal > 1000)  sendVal = 50;
    motor.setPower(sendVal);
    delayMicroseconds(100);
    receiveVal = motor.getPower();

    numSent++;
    if (receiveVal == 17) {
      numDelayErrors++;
    } else if (receiveVal != sendVal) {
      numOthErrors++;
    }

    // Print if error
    if (numSent > 1000) {
      Serial.print(numDelayErrors / float(numSent) * 100.0);
      Serial.print("%, ");
      Serial.print(numOthErrors / float(numSent) * 100.0);
      Serial.println("%");

      numOthErrors = 0;
      numDelayErrors = 0;
      numSent = 0;
    }
    
    lastUpdate = micros();
  }
}  // end of loop
