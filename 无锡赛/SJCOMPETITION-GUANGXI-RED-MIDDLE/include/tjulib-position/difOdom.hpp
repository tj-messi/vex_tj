#pragma once
#include "Math-Functions.h"
#include "vex.h"
#include "Position.hpp"

namespace tjulib{

    using namespace vex;
    typedef double T;

    class Dif_Odom : public Position{

    private:
        
        Math OMath;
        inertial& imu;
        std::vector<vex::motor*> &_leftMotors;
        std::vector<vex::motor*> &_rightMotors;

        Point localDeltaPoint{0,0,0};

        T r_motor = 0;

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


        void OdomRun(){
            T LeftSideMotorPosition=0, RightSideMotorPosition = 0;
            T LastLeftSideMotorPosition=0, LastRightSideMotorPosition = 0;
            
            while(true) {         
                // 获取imu读数
                globalPoint.angle = OMath.getRadians(imu.angle());
                T DeltaHeading = globalPoint.angle - prevGlobalPoint.angle;
                // 读取电机转角
                LeftSideMotorPosition = 0;
                RightSideMotorPosition = 0;
                for(motor* m : _leftMotors) LeftSideMotorPosition += OMath.getRadians(m->position(deg)) / _leftMotors.size()* r_motor;
                for(motor* m : _rightMotors) RightSideMotorPosition += OMath.getRadians(m->position(deg)) / _rightMotors.size()* r_motor;
                
                T LeftSideDistance=(LeftSideMotorPosition - LastLeftSideMotorPosition); 
                T RightSideDistance=(RightSideMotorPosition-LastRightSideMotorPosition);
                LeftBaseDistance = LeftSideMotorPosition*r_motor; 
                RightBaseDistance = RightSideMotorPosition*r_motor;
                // printf("LeftSideDistance : %lf RightSideDistance : %lf\n", LeftSideDistance, RightSideDistance);
                // printf("LeftBaseDistance : %lf RightBaseDistance : %lf\n", LeftBaseDistance, RightBaseDistance);

                // printf("%lf\n",LeftBaseDistance);
                // 计算相对坐标系中的变化
                T CenterRidus = DeltaHeading !=0 ? (LeftSideDistance + RightSideDistance)/(2*DeltaHeading) : MAXFLOAT;
                localDeltaPoint.x = CenterRidus * (1-cos(DeltaHeading));
                localDeltaPoint.y = CenterRidus * sin(DeltaHeading);
                
                // 计算全局坐标中的变化
                globalPoint.x = prevGlobalPoint.x - localDeltaPoint.x * cos(prevGlobalPoint.angle) + localDeltaPoint.y * sin(prevGlobalPoint.angle);
                globalPoint.y = prevGlobalPoint.y + localDeltaPoint.x * sin(prevGlobalPoint.angle) + localDeltaPoint.y * cos(prevGlobalPoint.angle);
                // 更新坐标
                prevGlobalPoint.y = globalPoint.y;
                prevGlobalPoint.x = globalPoint.x;
                prevGlobalPoint.angle = globalPoint.angle;
                LastLeftSideMotorPosition = LeftSideMotorPosition;
                LastRightSideMotorPosition = RightSideMotorPosition;     
                // 调试信息打印
            //   printf("x : %lf , y : %lf  , heading : %lf imu : %lf\n",globalPoint.x  ,globalPoint.y  , globalPoint.angle, imu.angle());
                // printf("GPS x : %lf cell , y : %lf cell , angle : %lf , rotation : %lf\n", GPS_.xPosition()/cell , GPS_.yPosition()/cell, GPS_.heading(),Math::getWrap360( imu.rotation()));
                this_thread::sleep_for(10);
            }

        }
        void setPosition(float newX, float newY, float newAngle = 114514) override{

            prevGlobalPoint.x = newX;
            prevGlobalPoint.y = newY;
            
            globalPoint.x = newX;
            globalPoint.y = newY;
            if(fabs(newAngle - 114514)>1){
                globalPoint.angle = newAngle;
                prevGlobalPoint.angle = newAngle;
            }
            
        }
        void executePosition() override{
            OdomRun();
        }
    };

}