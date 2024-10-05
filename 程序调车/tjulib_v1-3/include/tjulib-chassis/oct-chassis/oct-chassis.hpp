#pragma once
#include "tjulib-chassis/oct-chassis/oct-straight.hpp"

namespace tjulib
{
    using namespace vex;

    class Oct_SmartChassis : public Oct_StraChassis{
    public:
        Oct_SmartChassis(std::vector<std::vector<vex::motor*>*> &_chassisMotors, pidControl *_motorpidControl, Position*_position, const T _r_motor,
         pidControl *curpid, pidControl *fwdpid, pidControl *turnpid,T car_width)
            : 
            Oct_BaseChassis(_chassisMotors, _motorpidControl, _position, _r_motor),
            Oct_StraChassis(_chassisMotors, _motorpidControl, _position,  _r_motor, fwdpid, turnpid)
        {};
    };
};