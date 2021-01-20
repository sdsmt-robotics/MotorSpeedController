#include "PID.h"
#include <avr/io.h>


PID::PID(double Kp,double Ki, double Kd, double N, 
        unsigned long sample_time)
{
    setPIDConsts(Kp, Ki, Kd, N, sample_time);
}

void PID::setPIDConsts(double Kp,double Ki, double Kd, double N, unsigned long sample_time)
{
    this -> Kd = Kd;
    this -> Kp = Kp;
    this -> Ki = Ki;
    this -> N  = N;
    this -> Ts = sample_time/1000000.0;
    calculateCoeffs();
}

void PID::calculateCoeffs()
{
    a0 = (1+ N * Ts);
    a1 = - (2 + N * Ts);
    a2 = 1;

    b0 = Kp * (1+N*Ts) + Ki * Ts * (1 + N * Ts) + Kd * N;
    b1 = -( Kp * ( 2 + N * Ts ) + Ki * Ts + 2 * Kd * N );
    b2 = Kp + Kd*N;

    ku1 = a1/a0;
    ku2 = a2/a0;

    ke0 = b0/a0;
    ke1 = b1/a0;
    ke2 = b2/a0;
}

double PID::calculatePID(double Input)
{
    this->Input = Input;
    /* If over riding this function, add output code here */


    e2 = e1;
    e1 = e0;
    u2 = u1;
    u1 = u0;
    u0_part = -ku1 * u1 - ku2 * u2 + ke1 * e1 + ke2 * e2;

    /* If over riding this function, Move all lines below to start of function */
    e0 = Setpoint - Input;

    u0 = u0_part + ke0 * e0;

    PID_output = u0;
    
    // Constrain the output
    if (PID_output > max) {
        ScaledPIDOutput = max;
    } else if (PID_output < min) {
        ScaledPIDOutput = min;
    } else {
        ScaledPIDOutput = PID_output;
    }

    return ScaledPIDOutput;


}

void PID::setLimits(int16_t min, int16_t max)
{
    this->min = min;
    this->max = max;
}

void PID::setTarget(double target) {
  this->Setpoint = target;
}

double PID::getTarget() {
  return Setpoint;
}

void PID::reset()
{
    e0 = 0;
    e1 = 0;
    e2 = 0;

    u0_part = 0;
    PID_output = 0;
    ScaledPIDOutput = 0;
    u0 = 0;
    u1 = 0;
    u2 = 0;

}
/*
ISR(TIMER1_COMPA_vect)
{
    Instance->calculatePID();
}
*/
