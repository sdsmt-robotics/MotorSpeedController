/**
 * Demo motor control cycling through various speeds.
 * 
 * Wiring:
 * SPI connections (MOSI, MISO, SCK, and SS). Can use a different pin for SS
 * if doing multiple motors.
 */


#include <SPI.h>
#include "Nidec24hController.h"

// Motor Controller class
Nidec24hController motor(SPI, SS);


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
  const int speeds[] = {900, 500, 1600, 400};
  const int numSpeeds = 4;
  static int speedIndex = 0;
  static unsigned long lastSpeedChange = millis();
  static unsigned long lastPrint = millis();
  
  static int power = 0; //power level

  //go to the next speed if past time
  if (millis() - lastSpeedChange > 3000) {
    speedIndex++;
    if (speedIndex >= numSpeeds) {
      speedIndex = 0;
    }
    
    motor.setSpeed(speeds[speedIndex]);
    
    lastSpeedChange = millis();
  }
  
  //Output current speed
  if (millis() - lastPrint > 20) {
    Serial.print(speeds[speedIndex]);
    Serial.print(",\t");
    Serial.println(motor.getSpeed());
    
    lastPrint = millis();
  }
}  // end of loop
