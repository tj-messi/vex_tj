#pragma once
// #include "filter/ekf.hpp"
#include "Math-Functions.h"
#include "vex.h"

// extern std::vector<vex::motor*> _leftMotors;
// extern std::vector<vex::motor*> _rightMotors;

namespace tjulib{

    using namespace vex;
    typedef double T;
    struct Point {
        T x, y, angle;
    };
    
    // 差速定位 —— 车头指向y轴方向
    class Dif_Odom{
    public:
        Point globalPoint{0,0,0}; //x, y, angle(Not Deciduous)
        Point prevGlobalPoint{0,0,0}; 
        T LeftBaseDistance =0, RightBaseDistance = 0;
    private:
        Math OMath;
        inertial& imu;
        std::vector<vex::motor*> &_leftMotors;
        std::vector<vex::motor*> &_rightMotors;

        Point localDeltaPoint{0,0,0};

        T r_motor = 5;

        void resetTotalDistance(){
            for(motor* m : _leftMotors)
                m->resetPosition();
            for(motor* m : _rightMotors)
                m->resetPosition();
        }
    public:
        Dif_Odom(std::vector<vex::motor*> &leftMotors, std::vector<vex::motor*> &rightMotors, T wheelCircumference, T r_motor,                 
                inertial& imu
                ) : _leftMotors(leftMotors), _rightMotors(rightMotors),
                OMath(wheelCircumference), r_motor(r_motor), imu(imu)
        {
            resetTotalDistance();
        }
        // angle in degree
        void setPosition(T newX, T newY, T newAngle){
            // resetTotalDistance();  // 不要更改电机编码器的值，连续变化的
            prevGlobalPoint.x = newX;
            prevGlobalPoint.y = newY;
            prevGlobalPoint.angle = newAngle;
            // 当前值在一次循环之后完全由之前的状态决定
            imu.setRotation(newAngle, deg);
        }
        void setXY(T newX, T newY){
            // resetTotalDistance();  // 不要更改电机编码器的值，连续变化的
            prevGlobalPoint.x = newX;
            prevGlobalPoint.y = newY;
        }

        int OdomRun(){
            T LeftSideMotorPosition=0, RightSideMotorPosition = 0;
            T LastLeftSideMotorPosition=0, LastRightSideMotorPosition = 0;
            while(true) {         
                // time_cur = Time.time(msec);
                // deltime = time_cur-time_last;
                // time_last=time_cur;
                // 角度定位
                globalPoint.angle = OMath.getRadians(imu.rotation());
                T DeltaHeading = globalPoint.angle - prevGlobalPoint.angle;
                // 更新左右轮距离
                for(motor* m : _leftMotors) LeftSideMotorPosition = OMath.getRadians(m->position(deg))/_leftMotors.size();
                for(motor* m : _rightMotors) RightSideMotorPosition = OMath.getRadians(m->position(deg))/_rightMotors.size();
                T LeftSideDistance=(LeftSideMotorPosition - LastLeftSideMotorPosition)*r_motor; 
                T RightSideDistance=(RightSideMotorPosition-LastRightSideMotorPosition)*r_motor;
                LeftBaseDistance = LeftSideMotorPosition*r_motor; RightBaseDistance = RightSideMotorPosition*r_motor;
                // 定位1
                // localDeltaPoint.x = (LeftSideDistance+RightSideDistance)/2;
                // 更新局部位置
                T CenterRidus = DeltaHeading?(LeftSideDistance + RightSideDistance)/(2*DeltaHeading) : MAXFLOAT;
                localDeltaPoint.x = CenterRidus*(1-cos(DeltaHeading));
                localDeltaPoint.y = CenterRidus*sin(DeltaHeading);
                // 坐标变换
                globalPoint.x = prevGlobalPoint.x + localDeltaPoint.x*cos(prevGlobalPoint.angle) - localDeltaPoint.y*sin(prevGlobalPoint.angle);
                globalPoint.y = prevGlobalPoint.y + localDeltaPoint.x*sin(prevGlobalPoint.angle) + localDeltaPoint.y*cos(prevGlobalPoint.angle);
                // 更新旧值
                prevGlobalPoint.x = globalPoint.x;
                prevGlobalPoint.y = globalPoint.y;
                prevGlobalPoint.angle = globalPoint.angle;
                LastLeftSideMotorPosition = LeftSideMotorPosition;
                LastRightSideMotorPosition = RightSideMotorPosition;        
                // 打印
                printf("x: %lf, y: %lf, heading: %lf\n",globalPoint.x,globalPoint.y, globalPoint.angle);
                this_thread::sleep_for(10);
            }
            return 0;
        }
    };

}