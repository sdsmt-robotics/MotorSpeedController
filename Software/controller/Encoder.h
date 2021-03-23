/* 
 * Header for the Encoder class.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"
#include <SimpleKalmanFilter.h>

class Encoder {
public:
    Encoder(int aPin, int bPin, int ticksPerRotation);
    //Encoder(int dioPin, int csPin, int clkPin);
  
    void init();
    int estimateSpeed();
    int getSpeed();
    int getFilteredSpeed();
    void invertDirection(bool invertDir);

    volatile int ticks = 0;
    unsigned long lastInterval = 0; //last interval between ticks

    volatile int speed; //current speed of the motor in rpm
    int filteredSpeed; //current filtered speed of the motor in rpm
    int tickCount = 0; //current filtered speed of the motor in rpm

    long lastAngle = 0;
    int ticksPerRotation = 360;
    static void isrA();
    static void isrB();
private:
    void tick(bool trigA);

    //Static stuff for interrupt handling
    //static void isrA();
    //static void isrB();
    static Encoder* instance;

    // Pins and pin registers
    int aPin, bPin;
    uint8_t* aPinRegister;
    uint8_t* bPinRegister;
    uint8_t aPinBit, bPinBit;

    bool invertDir = false; // Track whether speed estimation should be negated

    volatile unsigned long lastTickTime; //time in microseconds of the last encoder tick
    volatile unsigned long lastEstTime; //time in microseconds of the tick preceding the last estimate

    // Conversion from (ticks / us) -> (rot / min)
    unsigned long tickConversion = 1000000ul * 60 / ticksPerRotation;

    // Intervals far dealing with issues
    unsigned long stoppedInterval = tickConversion / 10;  // < 10 rpm
    unsigned long minTimeoutInterval = tickConversion / 200;// < 200 rpm

    //Filter speedFilter;  //filter for the motor speed

    /*
     SimpleKalmanFilter(e_mea, e_est, q);
     e_mea: Measurement Uncertainty 
     e_est: Estimation Uncertainty 
     q: Process Noise
     */
    SimpleKalmanFilter speedFilter;

    //AS5134 encoder;
};

#endif
