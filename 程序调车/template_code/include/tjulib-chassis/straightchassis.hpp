#pragma once
#include "vex.h"
#include "tjulib-position/PositionStrategy.hpp"
#include "pidControl.hpp"
#include "basechassis.hpp"
#include <cmath>
extern double zero_drift_error;
namespace tjulib
{
    using namespace vex;

    class StraChassis : virtual public ChassisMotionFunc{
    protected:
        pidControl *fwdControl = NULL;
        pidControl *turnControl = NULL;
    private:
        const double PI = 3.14159265358979323846;

    protected:
    
    
    public:
        StraChassis(std::vector<vex::motor*> &leftMotors, std::vector<vex::motor*> &rightMotors, 
                 Position *odom, pidControl *fwdpid, pidControl *straightlinepid, pidControl *turnpid)
            : ChassisMotionFunc(leftMotors, rightMotors, straightlinepid, odom), 
            fwdControl(fwdpid), turnControl(turnpid)
        {};
        int cnt = 0;
        
        void turnToAngle(double angle, T maxSpeed, double maxtime_ms, int fwd = 1){
            timer mytime;
            mytime.clear();
            double totaltime = 0;
            T finalTurnSpeed = 20;

            double targetDeg = Math::getWrap360(angle); // Obtain the closest angle to the target position
            
            double currentAngle = Math::getWrap360(imu.rotation());
            double prev_speed = finalTurnSpeed;
            int init =0;

            while(targetDeg >= 360)
                targetDeg -= 360; 

            turnControl->resetpid();
            cnt = 0;
            while (!turnControl->overflag() || (fabs(currentAngle-targetDeg) >= 2)) // If within acceptable distance, PID output is zero.
            {
                
                if(totaltime=mytime.time(msec)>=maxtime_ms){
                    break;
                }
                if(std::fabs(targetDeg - currentAngle) < turnControl->params->errorThreshold && finalTurnSpeed <= turnControl->params->minSpeed){
                    turnControl->cnt++;
                }

                currentAngle = imu.rotation() - zero_drift_error;
                if(fwd)
                    angleWrap(targetDeg, currentAngle);  // 大小角调整
                finalTurnSpeed = turnControl->pidCalcu(targetDeg, maxSpeed, currentAngle); // Plug angle into turning PID and get the resultant speed
        
                if(finalTurnSpeed*prev_speed<0&& init > 0){
                    maxSpeed *= 0.2;
                }
                init = 1;
     
                prev_speed = finalTurnSpeed;
                
                VRUN(-finalTurnSpeed, finalTurnSpeed);
                task::sleep(5);
            }

            turnControl->resetpid();

            VRUN(0, 0);
            setStop(vex::brakeType::brake);
        }
        
        void DistanceSensorMove(T mms, T maxSpeed, double maxtime_ms = 5000, int fwd = 1){
            timer mytime;
            mytime.clear();
            T finalFwdSpeed = 20;
            T targetDistant = mms;
            T startError = DistanceSensor.objectDistance(mm);
            fwdControl->resetpid();
           
            while (!fwdControl->overflag()) // If within acceptable distance, PID output is zero.
            {
                if(targetDistant<=5 && finalFwdSpeed <= 15){
                    fwdControl->cnt++;
                }
                  printf("error: %lf distance: %lf \n", targetDistant, DistanceSensor.objectDistance(mm));
                // printf("cnt %lf \n", fwdControl->cnt);
                // printf("speed %lf \n", finalFwdSpeed);
                targetDistant = -(mms - fabs(DistanceSensor.objectDistance(mm)) ); // Obtain the closest angle to the target position
                
                
                if(mytime.time(msec)>=maxtime_ms){
                    break;
                }
                finalFwdSpeed = fwdControl->pidCalcu(targetDistant, maxSpeed); // Plug angle into turning PID and get the resultant speed 
                
                if(!finalFwdSpeed) finalFwdSpeed = 2;
                if(!fwd) finalFwdSpeed = -finalFwdSpeed;
                VRUN(finalFwdSpeed, finalFwdSpeed);

                task::sleep(5);
            }
            VRUN(0, 0);
            fwdControl->resetpid();
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

    
        
        // distance of base
        void moveInches(T inches, T maxSpeed, double maxtime_ms = 5000, int fwd = 1){
            timer mytime;
            mytime.clear();
            T finalFwdSpeed = 20;
            T targetDistant = inches;
            T startError = (position->LeftBaseDistance + position->RightBaseDistance) / 2;
            fwdControl->resetpid();
           
            while (!fwdControl->overflag()) // If within acceptable distance, PID output is zero.
            {
                targetDistant = inches - fabs(startError - (position->LeftBaseDistance + position->RightBaseDistance)/2) ; // Obtain the closest angle to the target position
                printf("error1 %lf \n", targetDistant);
                if(std::fabs(targetDistant)<=fwdControl->params->errorThreshold && finalFwdSpeed <= fwdControl->params->minSpeed){
                    fwdControl->cnt++;
                }

                // printf("cnt %lf \n", fwdControl->cnt);
                // printf("speed %lf \n", finalFwdSpeed);
                
                
                if(mytime.time(msec)>=maxtime_ms){
                    break;
                }
                finalFwdSpeed = fwdControl->pidCalcu(targetDistant, maxSpeed); // Plug angle into turning PID and get the resultant speed 
                
                if(!finalFwdSpeed) finalFwdSpeed = 2;
                if(!fwd) finalFwdSpeed = -finalFwdSpeed;
                VRUN(finalFwdSpeed, finalFwdSpeed);
               // VRUN(finalFwdSpeed, finalFwdSpeed);

                task::sleep(35);
            }
            VRUN(0, 0);
            fwdControl->resetpid();
        }
    };
};