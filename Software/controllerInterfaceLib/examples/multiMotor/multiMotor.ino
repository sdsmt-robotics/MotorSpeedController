/**
 * Demo motor control setting speeds rather quickly to obtain an oscilating speed.
 * 
 * Wiring:
 * SPI connections (MOSI, MISO, SCK, and SS). Can use a different pin for SS
 * if doing multiple motors.
 */


#include <SPI.h>
#include "Nidec24hController.h"

#define MOTOR_PIN_1 SS
#define MOTOR_PIN_2 6
#define MOTOR_PIN_3 7

// Motor Controller class (check that i am using this right
Nidec24hController motor1(SPI, MOTOR_PIN_1);
Nidec24hController motor2(SPI, MOTOR_PIN_2);
Nidec24hController motor3(SPI, MOTOR_PIN_3);

const unsigned long UPDATE_INTERVAL = 1000;
const unsigned long POWER = 200;


//===SETUP=============================
void setup (void) {
  Serial.begin (115200);

  // Initialize the SPI communications
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);

  // Initialize the motor control
  motor1.init();
  motor2.init();
  motor3.init();

  Serial.println("Begin Multi Motor Test");
}

//===LOOP=============================
void loop (void) {
  Serial.println("Run Motor 1");
  motor1.setPower(POWER);
  motor2.setPower(0);
  motor3.setPower(0);
  delay(UPDATE_INTERVAL);
  
  Serial.println("Run Motor 2");
  motor1.setPower(0);
  motor2.setPower(POWER);
  motor3.setPower(0);
  delay(UPDATE_INTERVAL);
  
  Serial.println("Run Motor 3");
  motor1.setPower(0);
  motor2.setPower(0);
  motor3.setPower(POWER);
  delay(UPDATE_INTERVAL);
  
  Serial.println("Run Motor 1 & 2");
  motor1.setPower(POWER);
  motor2.setPower(POWER);
  motor3.setPower(0);
  delay(UPDATE_INTERVAL);
  
  Serial.println("Run Motor 1 & 3");
  motor1.setPower(POWER);
  motor2.setPower(0);
  motor3.setPower(POWER);
  delay(UPDATE_INTERVAL);
  
  Serial.println("Run Motor 2 & 3");
  motor1.setPower(0);
  motor2.setPower(POWER);
  motor3.setPower(POWER);
  delay(UPDATE_INTERVAL);
  
  Serial.println("Run Motor 1 & 2 & 3");
  motor1.setPower(POWER);
  motor2.setPower(POWER);
  motor3.setPower(POWER);
  delay(UPDATE_INTERVAL);
}  // end of loop
