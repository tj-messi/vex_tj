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
    
    // ���ٶ�λ ���� ��ͷָ��y�᷽��
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
            // resetTotalDistance();  // ��Ҫ���ĵ����������ֵ�������仯��
            prevGlobalPoint.x = newX;
            prevGlobalPoint.y = newY;
            prevGlobalPoint.angle = newAngle;
            // ��ǰֵ��һ��ѭ��֮����ȫ��֮ǰ��״̬����
            imu.setRotation(newAngle, deg);
        }
        void setXY(T newX, T newY){
            // resetTotalDistance();  // ��Ҫ���ĵ����������ֵ�������仯��
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
                // �Ƕȶ�λ
                globalPoint.angle = OMath.getRadians(imu.rotation());
                T DeltaHeading = globalPoint.angle - prevGlobalPoint.angle;
                // ���������־���
                for(motor* m : _leftMotors) LeftSideMotorPosition = OMath.getRadians(m->position(deg))/_leftMotors.size();
                for(motor* m : _rightMotors) RightSideMotorPosition = OMath.getRadians(m->position(deg))/_rightMotors.size();
                T LeftSideDistance=(LeftSideMotorPosition - LastLeftSideMotorPosition)*r_motor; 
                T RightSideDistance=(RightSideMotorPosition-LastRightSideMotorPosition)*r_motor;
                LeftBaseDistance = LeftSideMotorPosition*r_motor; RightBaseDistance = RightSideMotorPosition*r_motor;
                // ��λ1
                // localDeltaPoint.x = (LeftSideDistance+RightSideDistance)/2;
                // ���¾ֲ�λ��
                T CenterRidus = DeltaHeading?(LeftSideDistance + RightSideDistance)/(2*DeltaHeading) : MAXFLOAT;
                localDeltaPoint.x = CenterRidus*(1-cos(DeltaHeading));
                localDeltaPoint.y = CenterRidus*sin(DeltaHeading);
                // ����任
                globalPoint.x = prevGlobalPoint.x + localDeltaPoint.x*cos(prevGlobalPoint.angle) - localDeltaPoint.y*sin(prevGlobalPoint.angle);
                globalPoint.y = prevGlobalPoint.y + localDeltaPoint.x*sin(prevGlobalPoint.angle) + localDeltaPoint.y*cos(prevGlobalPoint.angle);
                // ���¾�ֵ
                prevGlobalPoint.x = globalPoint.x;
                prevGlobalPoint.y = globalPoint.y;
                prevGlobalPoint.angle = globalPoint.angle;
                LastLeftSideMotorPosition = LeftSideMotorPosition;
                LastRightSideMotorPosition = RightSideMotorPosition;        
                // ��ӡ
                printf("x: %lf, y: %lf, heading: %lf\n",globalPoint.x,globalPoint.y, globalPoint.angle);
                this_thread::sleep_for(10);
            }
            return 0;
        }
    };

}