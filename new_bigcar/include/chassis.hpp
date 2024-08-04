#pragma once
#include "vex.h"
#include "difOdom.hpp"
#include "pidControl.hpp"
#include <cmath>
extern double zero_drift_error;

namespace tjulib
{
    using namespace vex;
    typedef double T;
    class Chassis{
    protected:
        std::vector<vex::motor*> &_leftMotors;
        std::vector<vex::motor*> &_rightMotors;
        const int deadzone = 5;//���òٿ�����
        vex::brakeType btype = vex::brakeType::brake;

    public:
        Chassis(std::vector<vex::motor*> &leftMotors, std::vector<vex::motor*> &rightMotors)
            : _leftMotors(leftMotors), _rightMotors(rightMotors) {}
        void setSpinPct(double Lspeed, double Rspeed){
            for (vex::motor* motor : (_leftMotors))
                motor->spin(vex::directionType::fwd, Lspeed, vex::pct);
            for (vex::motor* motor : (_rightMotors))
                motor->spin(vex::directionType::fwd, Rspeed, vex::pct);            
        }
        void VRUN(T Lspeed, T Rspeed){
            for (vex::motor *motor : (_leftMotors))
                vexMotorVoltageSet(motor->index(), Lspeed * 12000.0 / 100.0);
            for (vex::motor *motor : (_rightMotors))
                vexMotorVoltageSet(motor->index(), Rspeed * 12000.0 / 100.0);            
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
            // Retrieve the necessary joystick values
            int leftY = Controller1.Axis3.position(percent);
            int rightX = Controller1.Axis1.position(percent);
            if (abs(leftY) < deadzone) leftY = 0;            
            if (abs(rightX) < deadzone) rightX = 0;
            // ����
            //leftY = leftY * 0.5;
            rightX = rightX * 0.7;
            VRUN(leftY+rightX, leftY-rightX);
            /*
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
            */
        }

    };

    class OdomChassis : public Chassis{
    protected:
        Dif_Odom *odom = NULL;
        const T PI = 3.14159265358979323846;
        const T toDegrees = 180.0 / PI; // ���ȳ��Ը�����תΪ�Ƕ�

    protected:
        T GetDistance(Point p1, Point p2){
            return sqrt((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y));
        }
        T GetDistance(Point target){
            Point car = odom->globalPoint;
            return GetDistance(car, target);
        }

        T getDegree(Point target)
        {
            T relativeX = target.x - odom->globalPoint.x;
            T relativeY = target.y - odom->globalPoint.y;

            T deg = toDegrees * atan2(relativeY, relativeX);
            
            if((relativeX > 0 && relativeY > 0) || (relativeX < 0 && relativeY > 0))
                deg += 0;
            else if((relativeX > 0 && relativeY < 0) || (relativeX < 0 && relativeY < 0))
                deg += 360;
            else
                ;
            return deg;
        }

        // cur is negative when turn right
        T GetCur(Point car, Point target){
            // car.angle is Heading (for axis y)
            T d = GetDistance(car, target);
            return d?-(2*(sin(car.angle)*(target.y-car.y)+cos(car.angle)*(target.x-car.x))/(d*d)) : MAXFLOAT;
            // delet the "-" to set reverse
        }
        T GetCur(Point target){
            Point car = odom->globalPoint;        
            // car.angle is Heading (for axis y)
            return GetCur(car, target);
            // cur is negative when turn right
            // delet the "-" to set reverse
        }   
    public:
        OdomChassis(std::vector<vex::motor*> &leftMotors, std::vector<vex::motor*> &rightMotors, Dif_Odom *odom)
            : Chassis(leftMotors, rightMotors) ,odom(odom){};
    };

    class CurChassis : virtual public OdomChassis{
    protected:
        T b; // car width
        pidControl *cur_pid = NULL;

    protected:
        int cur_MotorSet(T cur, T V, bool fwd = true)
        {
            // if(cur < 0.0001) cur = 0;
            // printf( "CUR: %.1f\n" ,cur);
            T LeftMotorVelocity = V*(1-cur*b/2);
            T RightMotorVelocity = V*(1+cur*b/2);
            if(fwd) setSpinPct(LeftMotorVelocity, RightMotorVelocity);
            else setSpinPct(-LeftMotorVelocity, -RightMotorVelocity);
            return 0;
        }

    public:
        CurChassis(std::vector<vex::motor*> &leftMotors, std::vector<vex::motor*> &rightMotors, 
                Dif_Odom *odom, pidControl *cur_pid, T car_width)
            : OdomChassis(leftMotors, rightMotors, odom), 
            cur_pid(cur_pid), b(car_width)
        {};
        // open loop with cur and time
        int CurTimeMove(T cur, T V, T maxtime, bool fwd = true)
        {
            timer mytime;
            mytime.clear();
            while(mytime.time(msec)<maxtime)
            {
                cur_MotorSet(cur, V, fwd);
                task::sleep(20);
            }
            setStop(btype);
            task::sleep(20);
            return 0;
        }
        
        // open loop with target and time
        int CurPMove(Point target, T V, 
            bool fwd = true, T maxtime = 5000)
        {
            timer mytime;
            mytime.clear();
            Point car = odom->globalPoint;
            T d = GetDistance(target);
            T last_d = 0;
            while(d > 0.5 && mytime.time(msec)<maxtime){
                // if(!fwd) car.angle = -car.angle;
                T cur = GetCur(car, target);
                // ��ֹ�ӽ�ʱcur����
                if(fabs(cur)>0.15) break;
                T b = 50; //����
                // if(fwd) cur_MotorSet(cur, V);
                // else cur_MotorSet(-cur, -V);    // curȡ�������־���ֵ�Ե���Vȡ�������˶�
                cur_MotorSet(cur, V, fwd);
                task::sleep(20);
                // ����
                last_d = d;
                car = odom->globalPoint;
            }
            setStop(btype); // brake
            task::sleep(10);
            return 0;
        }
        
        // close loop with target and pid
        int CurPIDMove(Point target, T maxV,
            bool fwd = true, T maxtime = 5000)
        {
            timer mytime;
            mytime.clear();
            T d = GetDistance(target);
            T V = 3;

            while (fabs(V) > 2 &&(mytime.time(msec)<maxtime)) // If within acceptable distance, PID output is zero.
            {
                d = GetDistance(target);
                V = cur_pid->pidCalcu(d, maxV);
                if(!V) V = 2;
                Point car = odom->globalPoint;
                T cur = GetCur(car, target);
                // ��ֹ�ӽ�ʱcur����
                if(fabs(cur)>0.15) break;
                cur_MotorSet(cur, V, fwd);

                task::sleep(5);
            }
            setStop(btype); // brake
            cur_pid->reset();
            task::sleep(10);
            return 0;
        }

    };

    class StraChassis : virtual public OdomChassis{
    protected:
        pidControl *fwdControl = NULL;
        pidControl *turnControl = NULL;
    private:
        const double PI = 3.14159265358979323846;

    protected:
    void angleWrap(double& targetDeg, double& currentAngle)
    {
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
    
    public:
        StraChassis(std::vector<vex::motor*> &leftMotors, std::vector<vex::motor*> &rightMotors, 
                Dif_Odom *odom, pidControl *fwdpid, pidControl *turnpid)
            : OdomChassis(leftMotors, rightMotors, odom), 
            fwdControl(fwdpid), turnControl(turnpid)
        {};
        int cnt = 0;
        void turnToAngle(double angle, T maxSpeed, double maxtime_ms, int fwd){
            timer mytime;
            mytime.clear();
            double totaltime = 0;
            T finalTurnSpeed = 20;

            double targetDeg = Math::getWrap360(angle); // Obtain the closest angle to the target position
            
            double currentAngle = Math::getWrap360(imu.rotation());
            double prev_speed = finalTurnSpeed;
            int init =0;

            if(!fwd)
                targetDeg += 180;
            while(targetDeg >= 360)
                targetDeg -= 360; 

            turnControl->resetpid();
            cnt = 0;
            //  || odom->deltaAngle > DELTA_ANGLE_EPO ����Ŀ�������
            while (!turnControl->overflag() || (fabs(currentAngle-targetDeg) >= 2)) // If within acceptable distance, PID output is zero.
            {
                // printf("targetDeg: %f \n", targetDeg);
                // printf("currentAngle: %f \n", currentAngle);
                // printf("speed: %f \n", finalTurnSpeed);
                // printf("cnt: %d \n", cnt);
                // printf("error: %f \n", std::fabs(targetDeg - currentAngle));
                
                if(totaltime=mytime.time(msec)>=maxtime_ms){
                    break;
                }
                if(std::fabs(targetDeg - currentAngle)<1 && finalTurnSpeed<=21){
                    turnControl->cnt++;
                }

                currentAngle = imu.rotation() - zero_drift_error;
                angleWrap(targetDeg, currentAngle);  // ��ȡ����Ƕȣ�����ֵ����
                finalTurnSpeed = turnControl->pidCalcu(targetDeg, maxSpeed, currentAngle); // Plug angle into turning PID and get the resultant speed
                //�ж��Ƿ����
                if(finalTurnSpeed*prev_speed<0&& init > 0){
                    maxSpeed *= 0.1;
                }
                init = 1;
                //���ϴ�
                prev_speed = finalTurnSpeed;
                
                VRUN(-finalTurnSpeed, finalTurnSpeed);
                task::sleep(5);
            }

            turnControl->resetpid();

            VRUN(0, 0);
            setStop(vex::brakeType::brake);
        }
        void turnToTarget(Point target, T maxSpeed, double maxtime_ms, int fwd){
            // we set robot angle as the angle between heading and the y axis
            // that is equal to the angle between x axis minus 90 degree
            // the function getDegree is used to calculate angle for x axis
            turnToAngle(getDegree(target)-90,maxSpeed, maxtime_ms, fwd);
        }
        void moveToTarget(Point target, T maxFwdSpeed, T maxTurnSpeed, double maxtime_ms, int fwd)
        {
            timer mytime;
            mytime.clear();
            double totaltime = 0;
            
            turnToTarget(target, maxTurnSpeed, maxtime_ms / 2, fwd);

            T finalFwdSpeed = 3;

            T targetDistant;
            while (fabs(finalFwdSpeed) > 2 &&(totaltime=mytime.time(msec)<maxtime_ms)) // If within acceptable distance, PID output is zero.
            {
                targetDistant = GetDistance(target); // Obtain the closest angle to the target position
                finalFwdSpeed = fwdControl->pidCalcu(targetDistant, maxFwdSpeed); // Plug angle into turning PID and get the resultant speed
                if(!finalFwdSpeed) finalFwdSpeed = 2;
                if(!fwd) finalFwdSpeed = -finalFwdSpeed;
                VRUN(finalFwdSpeed, finalFwdSpeed);

                task::sleep(5);
            }
            VRUN(0, 0);
            fwdControl->reset();
        }

        
        // Point to Point
        void moveInches2(T inches, T maxSpeed, double maxtime_ms = 5000, int fwd = 1){
            timer mytime;
            mytime.clear();
            T finalFwdSpeed = 3;
            T targetDistant = inches;
            Point startPoint = odom->globalPoint;
            while (fabs(finalFwdSpeed) > 2 &&(mytime.time(msec)<maxtime_ms)) // If within acceptable distance, PID output is zero.
            {
                targetDistant = inches - GetDistance(startPoint) ; // Obtain the closest angle to the target position
                finalFwdSpeed = fwdControl->pidCalcu(targetDistant, maxSpeed); // Plug angle into turning PID and get the resultant speed
                if(!finalFwdSpeed) finalFwdSpeed = 2;
                if(!fwd) finalFwdSpeed = -finalFwdSpeed;
                VRUN(finalFwdSpeed, finalFwdSpeed);

                task::sleep(5);
            }
            VRUN(0, 0);
            fwdControl->reset();
        }
        
        // distance of base
        void moveInches(T inches, T maxSpeed, double maxtime_ms = 5000, int fwd = 1){
            timer mytime;
            mytime.clear();
            T finalFwdSpeed = 20;
            T targetDistant = inches;
            T startError = (odom->LeftBaseDistance+odom->RightBaseDistance)/2;
            fwdControl->resetpid();
           
            while (!fwdControl->overflag()) // If within acceptable distance, PID output is zero.
            {
                if(targetDistant<=1 && finalFwdSpeed<=32){
                    fwdControl->cnt++;
                }
                  printf("error: %lf \n", std::fabs(targetDistant));
                // printf("cnt %lf \n", fwdControl->cnt);
                // printf("speed %lf \n", finalFwdSpeed);
                targetDistant = inches - fabs(startError - (odom->LeftBaseDistance+odom->RightBaseDistance)/2) ; // Obtain the closest angle to the target position
                
                
                if(mytime.time(msec)>=maxtime_ms){
                    break;
                }
                finalFwdSpeed = fwdControl->pidCalcu2(targetDistant, maxSpeed); // Plug angle into turning PID and get the resultant speed 
                
                if(!finalFwdSpeed) finalFwdSpeed = 2;
                if(!fwd) finalFwdSpeed = -finalFwdSpeed;
                VRUN(finalFwdSpeed, finalFwdSpeed);

                task::sleep(5);
            }
            VRUN(0, 0);
            fwdControl->resetpid();
        }
    };
    
    // ����������̳У�����ֻ��һ��OdomChassis���ڴ�ӳ��
    // right basement: Hypotenuse and Leg Chassis Control
    class HLChassis : public CurChassis, public StraChassis{
    public:
        HLChassis(std::vector<vex::motor*> &leftMotors, std::vector<vex::motor*> &rightMotors, 
                Dif_Odom *odom, pidControl *curpid, pidControl *fwdpid, pidControl *turnpid, T car_width)
            : OdomChassis(leftMotors, rightMotors, odom), 
            CurChassis(leftMotors, rightMotors, odom, curpid, car_width),
            StraChassis(leftMotors, rightMotors, odom, fwdpid, turnpid)
        {};
    };

} // namespace tjulib
