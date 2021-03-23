/**
 * Discrete PID class to calculate an output for a process given an input.
 * 
 * Reference: https://www.scilab.org/discrete-time-pid-controller-implementation
 */

#include "PID.h"

/**
* @brief Description:
* Constructor for the PID class.
*
* @param Kp - double: The proportional gain term of the PID loop.
* @param Ki - double: The integral gain term of the PID loop.
* @param Kd - double: The derivative gain term of the PID loop.
* @param N - double: The low pass filtering amount.
* @param sample_time - unsigned long: The sample time.
*
* @return none.
*/
PID::PID(double Kp, double Ki, double Kd, double N,
        unsigned long sample_time)
{
    setPIDConsts(Kp, Ki, Kd, N, sample_time);
    reset();
}

/**
* @brief Description:
* Sets new PID constants.
*
* @param Kp - double: The proportional gain term of the PID loop.
* @param Ki - double: The integral gain term of the PID loop.
* @param Kd - double: The derivative gain term of the PID loop.
* @param N - double: The low pass filtering amount.
* @param sample_time - unsigned long: The sample time.
*
* @return none.
*/
void PID::setPIDConsts(double Kp, double Ki, double Kd, double N,
        unsigned long sample_time)
{
    this->Kd = Kd;
    this->Kp = Kp;
    this->Ki = Ki;
    this->N = N;
    this->Ts = sample_time / 1000000.0;
    calculateCoeffs();

    return;
}

/**
* @brief Description:
* Recalculate the PID coefficients for the calculations.
*
* @return none.
*/
void PID::calculateCoeffs() 
//https://www.scilab.org/discrete-time-pid-controller-implementation
{
    //intimidate calculations
    a0 = (1 + N * Ts);
    a1 = -(2 + N * Ts);
    a2 = 1;

    b0 = Kp * (1 + N * Ts) + Ki * Ts * (1 + N * Ts) + Kd * N;
    b1 = -(Kp * (2 + N * Ts) + Ki * Ts + 2 * Kd * N);
    b2 = Kp + Kd * N;

    //useful values
    derivitve_feedback_multiplayer = a1 / a0;        //(Ku1)
    second_derivitve_feedback_multiplayer = a2 / a0; //(Ku2)

    input_multiplayer = b0 / a0;                  //(Ke0)
    derivitve_input_multiplayer = b1 / a0;        //(Ke1)
    second_derivitve_input_multiplayer = b2 / a0; //(Ke2)

    return;
}

/**
* @brief Description:
* Update PID calculations and get a new PID output.
*
* @param Input - double: The input of the PID calculation.
*
* @return double: PID output.
*/
double PID::calculatePID(double Input)
{

    //update for the last three inputs
    last_error = previous_error;
    previous_error = current_error;
    current_error = target - Input;

    last_power = previous_power;
    previous_power = current_power;

    //calculate the new value
    current_power =
        - derivitve_feedback_multiplayer * previous_power 
        - second_derivitve_feedback_multiplayer * last_power 
        + input_multiplayer * current_error 
        + derivitve_input_multiplayer * previous_error 
        + second_derivitve_input_multiplayer * last_error;

    // Constrain the output
    if (current_power > max)
    {
        return max;
    }
    else if (current_power < min)
    {
        return min;
    }

    return current_power;
}

/**
* @brief Description:
* Sets the limits for the PID output.
*
* @param min - int16_t: The min value for the PID output.
* @param max - int16_t: The max value for the PID output.
*
* @return none.
*/
void PID::setLimits(int16_t min, int16_t max)
{
    this->min = min;
    this->max = max;

    return;
}

/**
* @brief Description:
* Sets the limits for the PID output.
*
* @param target - double: Set the PID target.
*
* @return none.
*/
void PID::setTarget(double target)
{
    this->target = target;

    return;
}

/**
* @brief Description:
* Gets the current setpoint.
*
* @return double: The current PID setpoint.
*/
double PID::getTarget()
{
    return target;
}

/**
* @brief Description:
* Resets all PID values for 
*
* @return none.
*/
void PID::reset()
{
    //set all values to zero

    // Reset errors
    current_error = 0;
    previous_error = 0;
    last_error = 0;

    // Reset outputs
    current_power = 0;
    previous_power = 0;
    last_power = 0;

    return;
}
