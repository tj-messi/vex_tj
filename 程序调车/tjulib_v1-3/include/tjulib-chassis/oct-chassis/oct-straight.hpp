#pragma once    
#include "tjulib-chassis/oct-chassis/oct-base.hpp"
#include "tjulib-position/odom.hpp"

extern double zero_drift_error;

namespace tjulib
{
    using namespace vex;

    class Oct_StraChassis : virtual public Oct_BaseChassis {
    protected:
        pidControl* fwdControl = NULL;      // 直线移动pid控制器   
        pidControl* turnControl = NULL;     // 转向pid控制器
    private:
        const double PI = 3.14159265358979323846;

    protected:

    public:
        Oct_StraChassis(std::vector<std::vector<vex::motor*>*>& _chassisMotors, pidControl* _motorpidControl, Position* _position, const T _r_motor, pidControl* _fwdpid, pidControl* _turnpid) :
            Oct_BaseChassis(_chassisMotors, _motorpidControl, _position, _r_motor),
            fwdControl(_fwdpid), turnControl(_turnpid) {}

        // angle : [-pi, pi] speed : pct
        void simpleMove(T speed, T angle, T sec, T gaptime = 10) {
            timer time;
            time.clear();
            T _speed = (speed / 100) * 850; // 850是默认的电机最大转速inches/s
            while (1) {
                if (time.time() >= sec * 1000) {
                    break;
                }
                T v_X = _speed * sin((angle / 180) * PI);
                T v_Y = _speed * cos((angle / 180) * PI);
                // 计算每个轮子的速度
                T v_lf = v_Y + v_X;
                T v_lb = v_Y - v_X;
                T v_rf = -v_Y + v_X;
                T v_rb = -v_Y - v_X;
                VRUNStable(v_lf, v_lb, v_rf, v_rb);
                task::sleep(gaptime);
            }
        }


        // 转向角pid
        void turnToAngle(double angle, T maxSpeed, double maxtime_ms, int fwd = 1) {
            timer mytime;
            mytime.clear();
            double totaltime = 0;
            T finalTurnSpeed = 20;

            double targetDeg = Math::getWrap360(angle); // Obtain the closest angle to the target position
            double currentAngle = Math::getWrap360(imu.rotation());

            double prev_speed = finalTurnSpeed;

            int init = 0;

            T error = optimalTurnAngle(targetDeg, currentAngle);

            turnControl->resetpid();

            while (!turnControl->overflag() || (fabs(error) >= 2)) // If within acceptable distance, PID output is zero.
            {

                if (totaltime = mytime.time(msec) >= maxtime_ms) {
                    break;
                }
                if (std::fabs(error) < turnControl->params->errorThreshold && finalTurnSpeed <= turnControl->params->minSpeed) {
                    turnControl->cnt++;
                }

                // 大小角调整
                currentAngle = imu.angle() - zero_drift_error;
                // 计算error
                if (fwd)
                    error = optimalTurnAngle(targetDeg, currentAngle);
                else
                    error = targetDeg - currentAngle;

                finalTurnSpeed = turnControl->pidCalcu(error, maxSpeed); // Plug angle into turning PID and get the resultant speed

                if (finalTurnSpeed * prev_speed < 0 && init > 0) {
                    maxSpeed *= 0.2;
                }
                init = 1;

                prev_speed = finalTurnSpeed;

                VRUN(finalTurnSpeed, finalTurnSpeed, finalTurnSpeed, finalTurnSpeed);

                task::sleep(5);
            }

            turnControl->resetpid();

            VRUN(0, 0, 0, 0);
            setStop(vex::brakeType::brake);
        }
        //直线360°平移
        void moveInches(T inches, T fwdAngle, T maxFwdSpeed, T maxtime_ms = 5000, T gaptime = 10, int fwd = 1) {

            timer mytime;
            mytime.clear();
            T finalFwdSpeed = 20;
            T targetDistant = inches;

            T targetFwdAngle = fwdAngle+imu.angle() - zero_drift_error;
            while (targetFwdAngle < 0)
                targetFwdAngle += 360;
            while(targetFwdAngle >= 360)
                targetFwdAngle -= 360;

            //目标位置记录
            T target_y = position->globalPoint.y + targetDistant * cos((targetFwdAngle / 180) * PI);
            T target_x = position->globalPoint.x + targetDistant * sin((targetFwdAngle / 180) * PI);
            //              while (1) {
             printf("position->globalPoint.x: %lf, position->globalPoint.y : %lf,position->globalPoint.angle: %lf\n",position->globalPoint.x ,position->globalPoint.y ,imu.angle() );
             printf("targetFwdAngle: %lf  target_x: %lf, target_y: %lf\n", targetFwdAngle, target_x, target_y);
            //  }
            fwdControl->resetpid();

            while (!fwdControl->overflag()) {
                targetDistant = GetDistance({ target_x, target_y });
                printf("targetDistant: %lf  fwdControl->cnt: %d\n  ", targetDistant, fwdControl->cnt);
                //printf("position->globalPoint.x: %lf, position->globalPoint.y : %lf\n",position->globalPoint.x ,position->globalPoint.y );
                if (targetDistant <= fwdControl->params->errorThreshold) {
                    fwdControl->cnt++;
                }
                if (mytime.time(msec) >= maxtime_ms) {
                    break;
                }
                finalFwdSpeed = fwdControl->pidCalcu(targetDistant, maxFwdSpeed);
                //限速
                if (!finalFwdSpeed) finalFwdSpeed = 2;
                if (!fwd) finalFwdSpeed = -finalFwdSpeed;

                T fwdSpeed_y = finalFwdSpeed * cos((fwdAngle / 180) * PI);
                T fwdSpeed_x = finalFwdSpeed * sin((fwdAngle / 180) * PI);
                // 计算每个轮子的速度
                T fwdSpeed_lf = fwdSpeed_y + fwdSpeed_x;
                T fwdSpeed_lb = fwdSpeed_y - fwdSpeed_x;
                T fwdSpeed_rf = -fwdSpeed_y + fwdSpeed_x;
                T fwdSpeed_rb = -fwdSpeed_y - fwdSpeed_x;
               // printf("targetDistant: %lf  fwdControl->cnt: %d  finalFwdSpeed: %lf  \n",
                 //   targetDistant, fwdControl->cnt, finalFwdSpeed);
                VRUN(fwdSpeed_lf, fwdSpeed_lb, fwdSpeed_rf, fwdSpeed_rb);
                task::sleep(gaptime);
            }
        }


        //写坐标点的moveInches
        void moveToTarget(Point target, T maxFwdSpeed, T maxtime_ms = 5000, T gaptime = 10, int fwd = 1) {
            T distance = GetDistance(target);
            T fwdAngle = 90 - getDegree(target) - position->globalPoint.angle;
            if (fwdAngle < 0)
                fwdAngle += 360;
            moveInches(distance, fwdAngle, maxFwdSpeed, maxtime_ms, gaptime, fwd);
        }

    };
};