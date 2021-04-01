/**
 * Demo motor control functions. Display a menu and take input over serial.
 * 
 * Wiring:
 * SPI connections (MOSI, MISO, SCK, and SS). Can use a different pin for SS
 * if doing multiple motors.
 */


#include <SPI.h>
#include "Nidec24hController.h"

// Menu handling
char menuChoice = 0;
int value = 0;

// Motor Controller class
Nidec24hController motor(SPI, SS);


//===SETUP=============================
void setup (void) {
  Serial.begin (115200);
  Serial.setTimeout(100);

  // Initialize the SPI communications
  SPI.begin ();
  SPI.setClockDivider(SPI_CLOCK_DIV32);

  // Initialize the motor control
  motor.init();
}

//===LOOP=============================
void loop (void)
{
  printMenu();

  // Wait for command
  while (Serial.available() < 1) { }
  menuChoice = Serial.read();

  switch (menuChoice) {
     case '1': // Set speed
      Serial.println("Enter speed: ");
      value = readSerialVal();
      motor.setSpeed(value);
      Serial.print("Speed set to ");
      Serial.println(value);
     break;
     case '2': // Set power
      Serial.println("Enter power: ");
      value = readSerialVal();
      motor.setPower(value);
      Serial.print("Power set to ");
      Serial.println(value);
     break;
     case '3': // Set brake
      Serial.println("Braking engaged.");
      motor.brake();
     break;
     case '4': // Set invert direction
      Serial.println("Set direction inverted (1 or 0): ");
      value = readSerialVal();
      motor.invertDirection(value);
      Serial.print("Invert set to: ");
      Serial.println(value);
     break;
     case '5': // Set invert direction
      Serial.println("Set timeout for the motor (ms): ");
      value = readSerialVal();
      motor.setTimeout(value);
      Serial.print("Timeout set to: ");
      Serial.println(value);
     break;
     case '6': // Get speed
      Serial.print("Current speed: ");
      value = motor.getSpeed();
      Serial.println(value);
     break;
     case '7': // Get power
      Serial.print("Current power: ");
      value = motor.getPower();
      Serial.println(value);
     break;
     case '8': // Get target speed
      Serial.print("Target speed: ");
      value = motor.getTargetSpeed();
      Serial.println(value);
     break;
     default:
      Serial.println("Invalid command.");
     break;
  }
  Serial.println();
}  // end of loop


//=====UTILITY funcitons=========================
void printMenu() {
  Serial.println("1 - Set Speed (rpm)");
  Serial.println("2 - Set Power (-1000 to 1000)");
  Serial.println("3 - Brake");
  Serial.println("4 - Set Direction Inverted");
  Serial.println("5 - Set Timeout");
  Serial.println("6 - Get Speed");
  Serial.println("7 - Get Power");
  Serial.println("8 - Get Target Speed");
  Serial.println();
}

int readSerialVal() {
  // Wait for value
  while(Serial.available() < 1) { }
  return Serial.parseInt();
}
