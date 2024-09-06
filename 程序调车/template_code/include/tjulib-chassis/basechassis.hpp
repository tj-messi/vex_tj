#pragma once
// 启用手动PID则打开，不启用则直接注释掉
//#define ManualPID
#include "vex.h"
#include "basechassis.hpp"
#include "tjulib-position/PositionStrategy.hpp"
#include "pidControl.hpp"
#include <cmath>
extern double zero_drift_error;
namespace tjulib
{
    using namespace vex;

    class Chassis{
    private:
        int straight_flag = 0;
        T current_yaw = 0;
        T initial_yaw = 0;
    protected:
        std::vector<vex::motor*> &_leftMotors;
        std::vector<vex::motor*> &_rightMotors;
        const int deadzone = 5; 
        vex::brakeType btype = vex::brakeType::brake;
        pidControl *StraightLineControl = NULL;   // 直线pid控制器
    
    public: 
        
        Chassis(std::vector<vex::motor*> &leftMotors, std::vector<vex::motor*> &rightMotors, pidControl *StraightLineControl)
            : _leftMotors(leftMotors), _rightMotors(rightMotors), StraightLineControl(StraightLineControl) {}

        void setSpinPct(double Lspeed, double Rspeed){
            for (vex::motor* motor : (_leftMotors))
                motor->spin(vex::directionType::fwd, Lspeed, vex::pct);
            for (vex::motor* motor : (_rightMotors))
                motor->spin(vex::directionType::fwd, Rspeed, vex::pct);            
        }

        // 夹角变换：作用是将俩角化到-360~360，并且符合顺逆时针小角的需求
        void angleWrap(double& targetDeg, double& currentAngle){
            bool flag = 0;
            // 0 < currentAngel < 360
            while(currentAngle < 0)
                currentAngle += 360;
            if(currentAngle >= 0){
                while(targetDeg < currentAngle){
                    flag = 1;
                    if(targetDeg + 360 < currentAngle)
                        targetDeg += 360;
                    else{
                        // 夹角是小角
                        if(currentAngle - targetDeg < 360 + targetDeg - currentAngle)
                            break;
                        else{
                            targetDeg += 360;
                            break;
                        }
                    }
                }
                if((targetDeg - currentAngle > 360 + currentAngle - targetDeg) && !flag)
                    currentAngle += 360;
            }
        }

        // 直接给电压的VRUN
        void VRUN(T Lspeed, T Rspeed){
            
            for (vex::motor *motor : (_leftMotors)){
                vexMotorVoltageSet(motor->index(), Lspeed * 12000.0 / 100.0);
                
            }
                
            for (vex::motor *motor : (_rightMotors)){
                vexMotorVoltageSet(motor->index(), Rspeed * 12000.0 / 100.0); 
            }
                           
        }

        // pid控制的直线VRUN
        void VRUNStraight(T Lspeed, T Rspeed, T max_delta_voltage){
            current_yaw = Math::getWrap360(imu.rotation());
            angleWrap(initial_yaw, current_yaw);
            // 获取两侧轮初始转速
            T leftMotorsvel = 0, rightMotorsvel = 0;
            for (vex::motor *motor : (_leftMotors)){
                leftMotorsvel += motor->velocity(dps) / _leftMotors.size();
            }

            for (vex::motor *motor : (_rightMotors)){
                rightMotorsvel += motor->velocity(dps) / _rightMotors.size();
            }
            T target_vel = (leftMotorsvel + rightMotorsvel) / 2;
            T delta_left_voltage = 0, delta_right_voltage = 0;
            // 计算速度
            T left_error = (target_vel - leftMotorsvel);
            T right_error = (target_vel - rightMotorsvel);
            //printf("left_error: %lf \n", left_error);
            //printf("right_error: %lf \n", right_error);
            
            if(std::fabs(current_yaw - initial_yaw) > 2){
           // if(std::fabs(target_vel) > 2300){
                printf("imu_current: %lf\n", current_yaw);
                printf("imu_initial: %lf\n", initial_yaw);
                left_error *=  0.075;
                right_error *=  0.075;
                // 通过速度差计算电压修正补偿（增量pid控制）
                delta_left_voltage = StraightLineControl->pidCalcu(left_error, max_delta_voltage);
                delta_right_voltage = StraightLineControl->pidCalcu(right_error, max_delta_voltage);
                //printf("delta_left_voltage: %lf \n", delta_left_voltage);
                //printf("delta_right_voltage: %lf \n", delta_right_voltage);
            }
        
            //printf("delta_left_voltage: %lf \n", delta_left_voltage);
           // printf("delta_right_voltage: %lf \n", delta_right_voltage);
            VRUN(Lspeed + delta_left_voltage, Rspeed + delta_right_voltage);
            
           /*
            current_yaw = Math::getWrap360(imu.rotation());
            angleWrap(initial_yaw, current_yaw);
            T delta_voltage = 0;
            printf("delta_left_voltage: %lf \n", std::fabs(current_yaw - initial_yaw));
            if(std::fabs(current_yaw - initial_yaw) > 3){
                
                // 通过速度差计算电压修正补偿（增量pid控制）
                delta_voltage = StraightLineControl->pidCalcu(initial_yaw, max_delta_voltage, current_yaw);
            }
        
            printf("delta_left_voltage: %lf \n", delta_voltage);
            VRUN(Lspeed - delta_voltage, Rspeed + delta_voltage);
              */
        }

        void setStop(vex::brakeType type){
            for (vex::motor *motor : (_leftMotors))
                motor->stop(type);
            for (vex::motor *motor : (_rightMotors))
                motor->stop(type);
        }
        void setBrakeType(vex::brakeType brake_type){btype = brake_type;}

        void simpleMove(double vel, double sec){
            setSpinPct(vel, vel);
            task::sleep(1000*sec);
            setSpinPct(0, 0);
            task::sleep(20);
        }

        void ArcadeDrive()
        {
            for (vex::motor *motor : (_leftMotors)){
               // printf("LeftMotor_vel : %lf \n", motor->velocity(dps));
            }
                
            for (vex::motor *motor : (_rightMotors)){
                //printf("RightMotor_vel : %lf \n", motor->velocity(dps));
            }
            // Retrieve the necessary joystick values
            int leftY = Controller1.Axis3.position(percent);
            int rightX = Controller1.Axis1.position(percent);
            if (abs(leftY) < deadzone) leftY = 0;            
            if (abs(rightX) < deadzone) rightX = 0;

            //rightX = rightX * 0.7;

#ifdef ManualPID
            if(leftY!=0 && rightX==0){
                VRUNStraight(leftY+rightX, leftY-rightX, leftY * 0.55);
            }else if(rightX!=0){
                VRUN(leftY+rightX, leftY-rightX);
                initial_yaw = Math::getWrap360(imu.rotation());
            }else{
                VRUN(0, 0);
                StraightLineControl->resetpid();
                initial_yaw = Math::getWrap360(imu.rotation());
            }
#else
            VRUN(leftY+rightX, leftY-rightX);
#endif

        }

    };

    class ChassisMotionFunc : public Chassis{
    protected:
        Position *position = NULL;
        const T PI = 3.14159265358979323846;
        const T toDegrees = 180.0 / PI; // ���ȳ��Ը�����תΪ�Ƕ�

    protected:

        

        // 获得两点距离
        T GetDistance(Point p1, Point p2){
            return sqrt((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y));
        }

        T GetDistance(Point target){
            Point car = position->globalPoint;
            return GetDistance(car, target);
        }

        // 获得两点之间的夹角
        T getDegree(Point target)
        {
            T relativeX = target.x - position->globalPoint.x;
            T relativeY = target.y - position->globalPoint.y;

            T deg = toDegrees * atan2(relativeY, relativeX);
            
            if((relativeX > 0 && relativeY > 0) || (relativeX < 0 && relativeY > 0))
                deg += 0;
            else if((relativeX > 0 && relativeY < 0) || (relativeX < 0 && relativeY < 0))
                deg += 360;
            else
                ;
            return deg;
        }


    public:
        ChassisMotionFunc(std::vector<vex::motor*> &leftMotors, std::vector<vex::motor*> &rightMotors,  pidControl *StraightLineControl, Position *_position)
            : Chassis(leftMotors, rightMotors, StraightLineControl) ,position(_position){};
    };
    

} // namespace tjulib
