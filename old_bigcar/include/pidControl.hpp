#pragma once
#include <math.h>
#include <stdio.h>
namespace tjulib
{
    typedef double T;
    struct pidParams
    {
        T kp, ki, kd;         // pid系数
        T integralActiveZone; // 积分限幅
        T errorThreshold;     // 终止误差

        pidParams(T kp, T ki, T kd, T integralActiveZone, T errorThreshold)
            : kp(kp), ki(ki), kd(kd), integralActiveZone(integralActiveZone), errorThreshold(errorThreshold){};
    };

    class pidControl
    {
    private:
        pidParams *params = NULL;
        double error = 0;      // Distance from target forward distance
        double lastError = 0;  // Keep track of last error for the derivative (rate of change)
        double integral = 0;   // Integral accumulates the error, speeding up if target is not reached fast enough
        double derivative = 0; // Derivative smooths out oscillations and counters sudden changes

    public:
        pidControl(pidParams *params) : params(params){};

        void reset();
        T pidCalcu(T target, T maxSpeed, T feedback = 0);
    };

}