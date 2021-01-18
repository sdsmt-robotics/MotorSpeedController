/* 
 * Class for Nidec 24H brushless motor.
 * This class provides functions for interfacing with the motor.
 * 
 * init() - initialize the motor class.
 * setPower(int power) - set the power level for the motor
 * brake() - set the motor to brake.
 * setDirection(Direction invertDir) - set the default the motor direction.
 * getDirection() - get the motor direction
 * 
 */

#include "Nidec24H.h"

/**
 * @brief Constructor for the class. PWM Pin must be 9 on atmega328!
 * 
 * @param dirPin - pin to toggle motor direction.
 * @param brakePin - pin to brake to toggle motor braking.
 */
Nidec24H::Nidec24H(int dirPin, int brakePin) 
    : dirPin(dirPin), brakePin(brakePin) {
}

/**
 * @brief Initialize the motor.
 */
void Nidec24H::init() {
    //Setup control pins
    pinMode(dirPin, OUTPUT);
    pinMode(brakePin, OUTPUT);

    // Set up the PWM pin
    initPwm();
    
    //Initialize the power
    setPower(0);
}

void Nidec24H::initPwm() {
    /*
    A bit of explanation for those who care...
    On the Arduino Uno, there are 8-bit and 16-bit timer/counters (0, 1, 2). Timer 0 (I think) is 
    special and used by millis() and delay(), so don't mess with it. Each timer is used 
    by 2-3 pins.
    Pins 8 and 9 use timer 1.
    Description of registers (n = timer #, x = pin letter):
      - TCCRnA/B/C: Timer/Counter control registers A, B, and C. Set timer source, mode, etc.
      - OCRnx - Output compare register. Value set for the pin to control duty cycle.
      - ICRn - Input caputer register. Used as the TOP (reset) value in phase correct PWM mode.
      - TCNTn - Timer/Counter for the clock. The current count for the cycle.

    OCnx is the output.
    */

    // Clear the control registers.
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    // Mode 10: phase correct PWM with ICR1 as Top (= F_CPU/2/16000)
    // OC1C as Non-Inverted PWM output
    // TOP = 1000
    // Bits set:
    //  - COM = 0b10 (clear output on match when upcounting, set output on match when downcounting)
    //  - WGM = 0b1010 (wave generation mode = phase correct PWM with ICR4 as the prescaler)
    //  - CS = 0b001 (Clock source 1, no prescaling)
    ICR1   = (F_CPU/8000)/2;      // Get the prescaler for 25k. Have to divide by two since phase correct.
    OCR1A  = 0;                    // Set initial duty cycle to 0%
    TCCR1A = _BV(COM1A1) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(CS10);

    maxPwmDuty = ICR1;
    pinMode(9, OUTPUT);
}

/**
 * Set the PWM to the given value.
 * 
 * @param value - duty cycle. (0 to 1000).
 */
void Nidec24H::setPwm(uint16_t value) {
  //Set the pwm duty cycle for pin 9
  OCR1A = value;
}

/**
 * @brief set whether the motor should be inverted.
 * 
 * @param invertDir - true if should be inverted, false otherwise
 */
void Nidec24H::invertDirection(bool invertDir) {
    this->invertDir = invertDir;

    // Update power since may need to invert
    setPower(power);
}

/**
 * @brief Run the motor at the given value (reverse is allowed).
 * 
 * @param power - the power level for the motor in the range 1000 to -1000.
 */
void Nidec24H::setPower(int power) {
    this->power = power;

    //stop the brake
    digitalWrite(brakePin, HIGH);

    //Get the desired direction.
    bool runInReverse = power > 0;

    // Set the correct direction based on the default
    if (invertDir == runInReverse) {
        digitalWrite(dirPin, LOW); // CW
    } else {
        digitalWrite(dirPin, HIGH); // CCW
    }
    
    //set the power.
    setPwm(maxPwmDuty - constrain(abs(power), 0, maxPwmDuty));
}

/**
 * @brief Get the current power level the motor is running at.
 * 
 * @return the current power level for the motor.
 */
int Nidec24H::getPower() {
    return power;
}

/**
 * @brief Engage the brake.
 */
void Nidec24H::brake() {
    //set to some power
    analogWrite(pwmPin, 50);

    //Turn on the brake (active low)
    digitalWrite(brakePin, LOW);
}
