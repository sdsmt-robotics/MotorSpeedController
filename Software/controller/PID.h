#ifndef PID_H
#define PID_H

#include<stdint.h>

class PID
{
    public:

        PID(double Kp,double Ki, double Kd, double N, unsigned long sample_time);

        double ScaledPIDOutput;
        double Input;
        double Setpoint;

        double calculatePID(double Input);

        void setPIDConsts(double Kp,double Ki, double Kd, double N, 
        unsigned long sample_time);
        void setLimits(int16_t min, int16_t max);
        void setTarget(double target);
        double getTarget();
        void reset();

    private:
        double Ts = 2500/1000000.0; // Sample time in seconds
        double a0, a1, a2, b0, b1, b2; // Need to define transfer function coefficients
        double e2,e1,e0,u2,u1,u0,u0_part;
        double ku1,ku2,ke0,ke1,ke2;


        int16_t min = 0;
        int16_t max = 100;

        double Kp = 10;
        double Ki = 1;
        double Kd = 1;
        double N = 20;

        double PID_output;



        void calculateCoeffs();
        

};


#endif
