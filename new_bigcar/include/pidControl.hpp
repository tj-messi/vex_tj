#pragma once

#include "vex.h"

typedef double T;

namespace tjulib
{
    struct pidParams
    {
        T kp, ki, kd;         // pid系数
        T integralActiveZone; // 积分限幅
        T errorThreshold;     // 终止误差
        int stop_num;         // 停止时震荡的次数
        pidParams(T kp, T ki, T kd, T integralActiveZone, T errorThreshold, int stop_num)
            : kp(kp), ki(ki), kd(kd), integralActiveZone(integralActiveZone), errorThreshold(errorThreshold), stop_num(stop_num){};
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
    
        int cnt = 0;
        
        pidControl(pidParams *params) : params(params){};

        T pidCalcu(T target, T maxSpeed, T feedback = 0);

        T pidCalcu2(T target, T maxSpeed, T feedback = 0);

        bool overflag();
        
        void reset();

        void resetpid();
    };

}