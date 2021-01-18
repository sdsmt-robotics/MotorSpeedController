/* 
 * Header for the Nidec 24H brushless motor.
 */

#ifndef AS5134_H
#define NIDEC24H_H

#include "Arduino.h"


class Nidec24H {
public:
    Nidec24H(int dirPin, int brakePin);
  
    void init();
    void setPower(int power);
    int getPower();
    void brake();
    void invertDirection(bool invertDir);

private:
    void setPwm(uint16_t value);
    void initPwm();

    uint16_t maxPwmDuty;  // Max pwm duty cycle value
    int pwmPin = 9;  // Hardcoded since this changing some PWM settings
    int dirPin, brakePin;
    bool invertDir = false;
    int power;
};

#endif
