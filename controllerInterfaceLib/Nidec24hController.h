/* 
 * Header for the Nidec24hController class.
 */

#ifndef NIDEC_24H_CONTROLLER_H
#define NIDEC_24H_CONTROLLER_H

#include "Arduino.h"
#include <SPI.h>

class Nidec24hController {
public:
    Nidec24hController(SPIClass &spi, int ssPin);
  
    void init();

    void setSpeed(int speed);
    void setPower(int power);
    void brake();
    void invertDirection(bool invertDir);

    int getSpeed();
    int getPower();
    int getTargetSpeed();

private:
    SPIClass &spi;
    int ssPin;

    const int TRANSFER_BYTE_DELAY = 50;

    int getVal(int command);
    void sendVal(int command, int value);
    int communicate(int command, int sendVal);
    
};

#endif
