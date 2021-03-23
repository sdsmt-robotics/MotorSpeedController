#ifndef PID_H
#define PID_H

#include<stdint.h>

class PID
{
    public:

        PID(float Kp,float Ki, float Kd, float N, unsigned long sample_time);

        float ScaledPIDOutput;
        float Input;
        float Setpoint;

        float calculatePID(float Input);

        void setPIDConsts(float Kp,float Ki, float Kd, float N, 
        unsigned long sample_time);
        void setLimits(int16_t min, int16_t max);
        void setTarget(float target);
        float getTarget();
        void reset();

    private:
        float Ts = 2500/1000000.0; // Sample time in seconds
        float a0, a1, a2, b0, b1, b2; // Need to define transfer function coefficients
        float e2,e1,e0,u2,u1,u0,u0_part;
        float ku1,ku2,ke0,ke1,ke2;


        int16_t min = 0;
        int16_t max = 100;

        float Kp = 10;
        float Ki = 1;
        float Kd = 1;
        float N = 20;

        float PID_output;



        void calculateCoeffs();
        

};


#endif
