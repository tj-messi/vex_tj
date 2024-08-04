#include "pidControl.hpp"

namespace tjulib
{
    T pidControl::pidCalcu(T target, T maxSpeed, T feedback)
    {
        T integralPowerLimit = 40 / params->ki; // 40 is the maximum power that integral can contribute

        T speed = 0;

        error = target - feedback; // Error is the distance from the target

        if (fabs(error) > params->errorThreshold) // Check if error is over the acceptable threshold
        {
            // Only accumulate error if error is within active zone
            if (fabs(error) < params->integralActiveZone)
            {
                integral = integral + error; // Accumulate the error to the integral
            }
            else
            {
                integral = 0;
            }
            // Limit the integral
            if (integral < -integralPowerLimit)
                integral = -integralPowerLimit;
            if (integral > integralPowerLimit)
                integral = integralPowerLimit;

            derivative = error - lastError; // Derivative is the change in error since the last PID cycle
            lastError = error; // Keep track of the error since last cycle

            speed = (params->kp * error) + (params->ki * integral) + (params->kd * derivative); // Multiply each variable by its tuning constant
             // Restrict the absolute value of speed to the maximum allowed value
            if (speed < -maxSpeed)
                speed = -maxSpeed;
            if (speed > maxSpeed)
                speed = maxSpeed;

            // make sure speed is always enough to overcome steady-state error
            if (speed > 0 && speed < 2)
            {
                speed = 2;
            }
            if (speed < 0 && speed > -2)
            {
                speed = -2;
            }
        }
        else
        {
            // If error threshold has been reached, set the speed to 0
            reset();
            speed = 0;
        }

        return speed; // Return the final speed value
    }
    void pidControl::reset(){
        integral = 0;
        derivative = 0;
        error = 0;
        lastError = 0;
    }

};