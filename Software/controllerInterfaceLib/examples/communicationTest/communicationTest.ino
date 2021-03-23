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
  SPI.setClockDivider(SPI_CLOCK_DIV32);

  // Initialize the motor control
  motor.init();

  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);

  pinMode(3, INPUT);

  Serial.println("Begin Oscilating Control");
}

//===LOOP=============================
void loop (void) {
  static int sendVal = 0;
  int receiveVal;
  static unsigned long lastUpdate = micros();
  static unsigned long numSent = 0;
  static unsigned long numProcErrors = 0;
  static unsigned long numDelayErrors = 0;
  static unsigned long numOthErrors = 0;


  // Get the power level from the speed controller
  if (micros() - lastUpdate > UPDATE_INTERVAL) {
    sendVal++;
    if (sendVal > 400)  sendVal = 50;
    motor.setPower(sendVal);
    delayMicroseconds(200);
    receiveVal = motor.getPower();

    numSent++;
    if (receiveVal == 0) {
      // Errors where command was received but not processed
      numProcErrors++;
    } else if ((sendVal - receiveVal <= 5 && sendVal - receiveVal > 0)  || (sendVal==50 && receiveVal==400)) {
      // Error where the value was not set in the controller fast enough
      numDelayErrors++;
    } else if (receiveVal != sendVal) {
      // Some other error
      numOthErrors++;
    }

    if (receiveVal != sendVal) {
      Serial.print(sendVal);
      Serial.print("->");
      Serial.println(receiveVal);
    }

    // Print if error
    if (numSent > 1000) {
      Serial.print("P: ");
      Serial.print(numProcErrors / float(numSent) * 100.0);
      Serial.print("%, D: ");
      Serial.print(numDelayErrors / float(numSent) * 100.0);
      Serial.print("%, O: ");
      Serial.print(numOthErrors / float(numSent) * 100.0);
      Serial.println("%");

      numProcErrors = 0;
      numOthErrors = 0;
      numDelayErrors = 0;
      numSent = 0;
    }
    
    lastUpdate = micros();
  }
}  // end of loop
