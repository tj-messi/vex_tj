#pragma once
#include "vex.h"
#include "pidControl.hpp"
#include "difOdom.hpp"

namespace tjulib{

    using namespace vex;

    typedef double T;
    // struct Point {
    //     float x, y, angle;
    // };
    // 继承
    #if 0
    class Dif_Control : public straightChassis{
    private:
        Dif_Odom *odom = NULL;
        pidControl *pid = NULL;
        T b;

    private:
    T GetDistance(Point p1, Point p2){
        return (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y);
    }
    T GetDistanceTo(Point target){
        Point car = odom->globalPoint;
        return GetDistance(car, target);
    }

    T GetCur(Point car, Point target){
        // car.angle is Heading (for axis y)
        T d = GetDistance(car, target);
        return d?-(2*(sin(car.angle)*(target.y-car.y)+cos(car.angle)*(target.x-car.x))/d) : MAXFLOAT;
        // cur is negative when turn right
        // delet the "-" to set reverse
    }
    T GetCurTo(Point target){
        Point car = odom->globalPoint;        
        // car.angle is Heading (for axis y)
        T d = GetDistance(car, target);
        return d?-(2*(sin(car.angle)*(target.y-car.y)+cos(car.angle)*(target.x-car.x))/d) : MAXFLOAT;
        // cur is negative when turn right
        // delet the "-" to set reverse
    }
    
    int cur_MotorSet(T cur, T V, bool fwd = true)
    {
        T LeftMotorVelocity = V*(1-cur*b/2);
        T RightMotorVelocity = V*(1+cur*b/2);
        if(fwd) setSpinPct(LeftMotorVelocity, RightMotorVelocity);
        else setSpinPct(-RightMotorVelocity, -LeftMotorVelocity);
        return 0;
    }

    public:
        Dif_Control(std::vector<vex::motor*> &leftMotors, std::vector<vex::motor*> &rightMotors, 
                Dif_Odom *odom, pidControl *pid, T car_width)
            : straightChassis(leftMotors, rightMotors), 
            odom(odom), pid(pid),
            b(car_width)
        {};

        // open loop with cur and time
        int simpleMove(T cur, T V, T maxtime)
        {
            timer mytime;
            mytime.clear();
            while(mytime.time(msec)<maxtime)
            {
                cur_MotorSet(cur, V);
                task::sleep(20);
            }
            setStop(brakeType::brake);
            task::sleep(20);
            return 0;
        }
        // open loop with target and time
        int MoveToTarget(Point target, T V, 
            bool fwd = true, T maxtime = 5000, brakeType _brakeType = brakeType::brake)
        {
            timer mytime;
            mytime.clear();
            Point car = odom->globalPoint;
            T d = GetDistanceTo(target);
            T last_d = 0;
            while(d > 3 && mytime.time(msec)<maxtime){
                // if(!fwd) car.angle = -car.angle;
                T cur = GetCur(car, target);
                // 防止接近时cur过大
                if(fabs(cur)>0.2) break;
                T b = 50; //车宽
                // if(fwd) cur_MotorSet(cur, V);
                // else cur_MotorSet(-cur, -V);    // cur取负左右轮绝对值对调，V取负反向运动
                cur_MotorSet(cur, V, fwd);
                task::sleep(20);
                // 更新
                last_d = d;
                car = odom->globalPoint;
            }
            setStop(_brakeType); // brake
            task::sleep(10);
            return 0;
        }
        // close loop with target and pid
        int PIDMoveToTarget(Point target, T maxV,
            bool fwd = true, T maxtime = 5000, brakeType _brakeType = brakeType::brake)
        {
            timer mytime;
            mytime.clear();
            T d = GetDistanceTo(target);
            T V = 3;

            while (std::abs(V) > 2 &&(mytime.time(msec)<maxtime)) // If within acceptable distance, PID output is zero.
            {
                d = GetDistanceTo(target); // Obtain the closest angle to the target position

                V = pid->pidCalcu(d, maxV); // Plug angle into turning PID and get the resultant speed
                if(!V) V = 2;
                // Turn in place towards the position
                Point car = odom->globalPoint;
                // if(!fwd) car.angle = -car.angle;
                T cur = GetCur(car, target);
                // if(fwd) cur_MotorSet(cur, V);
                // else cur_MotorSet(-cur, -V);    // cur取负左右轮绝对值对调，V取负反向运动
                cur_MotorSet(cur, V, fwd);

                task::sleep(5);
            }
            setStop(_brakeType); // brake
            task::sleep(10);
            return 0;
        }
    }; // end of class Dif_Control
    #endif

    #if 1
    class Dif_Control{
    private:
        Dif_Odom *odom = NULL;
        pidControl *pid = NULL;
        std::vector<vex::motor*> _leftMotors;
        std::vector<vex::motor*> _rightMotors;

        T b;
        const int deadzone = 5;//设置死区

    private:
    T GetDistance(Point p1, Point p2){
        return (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y);
    }
    T GetDistanceTo(Point target){
        Point car = odom->globalPoint;
        return GetDistance(car, target);
    }

    T GetCur(Point car, Point target){
        // car.angle is Heading (for axis y)
        T d = GetDistance(car, target);
        return d?-(2*(sin(car.angle)*(target.y-car.y)+cos(car.angle)*(target.x-car.x))/d) : MAXFLOAT;
        // cur is negative when turn right
        // delet the "-" to set reverse
    }
    T GetCurTo(Point target){
        Point car = odom->globalPoint;        
        // car.angle is Heading (for axis y)
        T d = GetDistance(car, target);
        return d?-(2*(sin(car.angle)*(target.y-car.y)+cos(car.angle)*(target.x-car.x))/d) : MAXFLOAT;
        // cur is negative when turn right
        // delet the "-" to set reverse
    }
    
    void setSpinPct(double Lspeed, double Rspeed){
        for (vex::motor* motor : (_leftMotors))
        {
            motor->spin(vex::directionType::fwd, Lspeed, vex::pct);
        }
        for (vex::motor* motor : (_rightMotors))
        {
            motor->spin(vex::directionType::fwd, Rspeed, vex::pct);
        }       
    }
    void setStop(vex::brakeType type)
    {
        for (vex::motor *motor : (_leftMotors))
        {
            motor->stop(type);
        }
        for (vex::motor *motor : (_rightMotors))
        {
            motor->stop(type);
        }
    }

    int cur_MotorSet(T cur, T V, bool fwd = true)
    {
        T LeftMotorVelocity = V*(1-cur*b/2);
        T RightMotorVelocity = V*(1+cur*b/2);
        if(fwd) setSpinPct(LeftMotorVelocity, RightMotorVelocity);
        else setSpinPct(-LeftMotorVelocity, -RightMotorVelocity);
        return 0;
    }

    public:
        Dif_Control(std::vector<vex::motor*> &leftMotors, std::vector<vex::motor*> &rightMotors, 
                Dif_Odom *odom, pidControl *pid, T car_width)
            : _leftMotors(leftMotors), _rightMotors(rightMotors), 
            odom(odom), pid(pid),
            b(car_width)
        {};

        // open loop with cur and time
        int simpleMove(T cur, T V, T maxtime)
        {
            timer mytime;
            mytime.clear();
            while(mytime.time(msec)<maxtime)
            {
                cur_MotorSet(cur, V);
                task::sleep(20);
            }
            setStop(brakeType::brake);
            task::sleep(20);
            return 0;
        }
        // open loop with target and time
        int MoveToTarget(Point target, T V, 
            bool fwd = true, T maxtime = 5000, brakeType _brakeType = brakeType::brake)
        {
            timer mytime;
            mytime.clear();
            Point car = odom->globalPoint;
            T d = GetDistanceTo(target);
            T last_d = 0;
            while(d > 3 && mytime.time(msec)<maxtime){
                // if(!fwd) car.angle = -car.angle;
                T cur = GetCur(car, target);
                // 防止接近时cur过大
                if(fabs(cur)>0.15) break;
                T b = 50; //车宽
                // if(fwd) cur_MotorSet(cur, V);
                // else cur_MotorSet(-cur, -V);    // cur取负左右轮绝对值对调，V取负反向运动
                cur_MotorSet(cur, V, fwd);
                task::sleep(20);
                // 更新
                last_d = d;
                car = odom->globalPoint;
            }
            setStop(_brakeType); // brake
            task::sleep(10);
            return 0;
        }
        // close loop with target and pid
        int PIDMoveToTarget(Point target, T maxV,
            bool fwd = true, T maxtime = 5000, brakeType _brakeType = brakeType::brake)
        {
            timer mytime;
            mytime.clear();
            T d = GetDistanceTo(target);
            T V = 3;

            while (fabs(V) > 2 &&(mytime.time(msec)<maxtime)) // If within acceptable distance, PID output is zero.
            {
                d = GetDistanceTo(target); // Obtain the closest angle to the target position

                V = pid->pidCalcu(d, maxV); // Plug angle into turning PID and get the resultant speed
                if(!V) V = 2;
                // Turn in place towards the position
                Point car = odom->globalPoint;
                // if(!fwd) car.angle = -car.angle;
                T cur = GetCur(car, target);
                // if(fwd) cur_MotorSet(cur, V);
                // else cur_MotorSet(-cur, -V);    // cur取负左右轮绝对值对调，V取负反向运动
                cur_MotorSet(cur, V, fwd);

                task::sleep(5);
            }
            setStop(_brakeType); // brake
            task::sleep(10);
            return 0;
        }
        // autonomous
        void ArcadeDrive()
        {
            // Retrieve the necessary joystick values
            int leftY = Controller1.Axis3.position(percent);
            int rightX = Controller1.Axis1.position(percent);
            if (abs(leftY) < deadzone)
            {
                leftY = 0;
            }
            if (abs(rightX) < deadzone)
            {
                rightX = 0;
            }
            // 限速
            //leftY = leftY * 0.5;
            rightX = rightX * 0.5;

            for (vex::motor *motor : (this->_leftMotors))
            {
                if(leftY ==0 && rightX == 0){
                    motor->stop(coast);
                }
                motor->spin(vex::directionType::fwd, leftY + rightX, vex::pct);
            }
            for (vex::motor *motor : (this->_rightMotors))
            {
                if(leftY == 0 && rightX == 0){
                    motor->stop(coast);
                }
                motor->spin(vex::directionType::fwd, leftY - rightX, vex::pct);
            }
        }
    }; // end of class Dif_Control
    #endif

} // end of namespace