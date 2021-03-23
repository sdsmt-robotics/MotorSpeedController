#ifndef PID_H
#define PID_H

#include <stdint.h>

class PID
{
public:
    PID(double Kp, double Ki, double Kd, double N, 
                        unsigned long sample_time); //constructor
    double calculatePID(double Input);              //calculate PID values
    void setPIDConsts(double Kp, double Ki, double Kd, double N,
                      unsigned long sample_time); //set the PID values
    void setLimits(int16_t min, int16_t max);     //set the power output limits
    void setTarget(double target);                //set the target value
    double getTarget();                           //get the set target
    void reset();                                 //reset all stored values

private:
    //sample time
    double Ts = 2500 / 1000000.0; // Sample time in seconds

    //transfer function constants
    double a0, a1, a2, b0, b1, b2;

    //error values over the past three loops
    double last_error, previous_error, current_error;

    //result values over the past three loops
    double last_power, previous_power, current_power;

    //constants based on PID values and time step
    double derivitve_feedback_multiplayer, 
                        second_derivitve_feedback_multiplayer;
    double input_multiplayer, derivitve_input_multiplayer, 
                        second_derivitve_input_multiplayer;

    //default values
    int16_t min = 0;
    int16_t max = 100;

    double Kp = 10;
    double Ki = 1;
    double Kd = 1;
    double N = 20;

    //recalculate the coefficients
    void calculateCoeffs();

    //main values
    double ScaledPIDOutput;
    double Setpoint;
};

#endif
