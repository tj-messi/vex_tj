#pragma once
#include "straightchassis.hpp"
#include "curchassis.hpp"

namespace tjulib
{
    using namespace vex;

    class SmartChassis : public CurChassis, public StraChassis{
    public:
        SmartChassis(std::vector<vex::motor*> &leftMotors, std::vector<vex::motor*> &rightMotors, 
                Position *position, pidControl *curpid, pidControl *fwdpid, pidControl *turnpid, pidControl *straightlinepid, T car_width)
            : ChassisMotionFunc(leftMotors, rightMotors, straightlinepid, position), 
            CurChassis(leftMotors, rightMotors, position, curpid, straightlinepid, car_width),
            StraChassis(leftMotors, rightMotors, position, fwdpid, straightlinepid, turnpid)
        {};
    };
};