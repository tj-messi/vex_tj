#pragma once

// 使用卡尔曼滤波减少误差则define，否则注释掉
//#define KalmanFilter
#include "Math-Functions.h"
#include "vex.h"
#include "Position.hpp"

#ifdef KalmanFilter
#include"tjulib-filter/ekf.hpp"
#include"tjulib-filter/ekf.cpp"
#endif

namespace tjulib{
    
    using namespace vex;
    T prevtheta = 0;
    T theta1, deltaTheta=0.0;
    T V = 0.0; 

    #ifdef KalmanFilter
    //滤波器
    ExtendedKalmanFilter ekfilter;
    #endif

    struct encoderType{
        float vertical, horizonal;
    };

    class Odom : public Position{
    private:

        //Math
        Math OMath;
        // 轮子中心相对于旋转中心的偏移量(作垂线)
        const T horizonalOffset; //inches
        const T verticalOffset;
        const T rotate_degree;
        Point localDeltaPoint{0,0,0}; //change in x, change in y
        Point globalDeltaPoint{0,0,0}; //change in x, change in y
        encoder& encoderVertical;
        encoder& encoderHorizonal;
        inertial& imu;
        //SENSOR VALUES
        //encoder
        encoderType encoderVal; //verticalEnc, horizonalEnc, backEnc
        encoderType prevEncoderVal; //prev verticalEnc, horizonalEnc, backEnc
        encoderType deltaEncoderVal; //change in verticalEnc, horizonalEnc, backEnc
        //angle
        float currentAngle;
        float prevAngle;
        float deltaAngle; //rad
        const int PI = 3.1415926;

        

    public: 
 
        Odom(T hOffset, T vOffset, T wheelCircumference, T rotate_degree, encoder& encoderVertical, encoder& encoderHorizonal, inertial& imu):
        horizonalOffset(hOffset), verticalOffset(vOffset), OMath(wheelCircumference), rotate_degree(rotate_degree),
        encoderVertical(encoderVertical), encoderHorizonal(encoderHorizonal), imu(imu){
           // encoderVertical.resetRotation(); 
           // encoderHorizonal.resetRotation(); 
            globalPoint = {0, 0, 0};
            prevGlobalPoint = {0, 0, 0};
            globalDeltaPoint = {0, 0, 0};

            //LOCAL COORDINATES
            localDeltaPoint = {0, 0};

            //SENSOR VALUES
            //encoder
            encoderVal = {0, 0}; //verticalEnc, horizonalEnc, backEnc
            prevEncoderVal = {0, 0};
            deltaEncoderVal = {0, 0};
            //angle
            currentAngle = 0.0;
            prevAngle = 0.0;
            deltaAngle = 0.0;
        }
#ifdef KalmanFilter
        // ekf
        void filteringData(){
            Eigen::Vector2d u;
            Eigen::Vector3d z;
            u << V, deltaTheta;
            //更新速度
            T dx=globalPoint.x-ekfilter.getState()(0),
            dy=globalPoint.y-ekfilter.getState()(1);
            V = sqrt(dx*dx+dy*dy);
            theta1 = atan2(dy,dx);
            deltaTheta = theta1-prevtheta;
            ekfilter.predict(u);
            z<<globalPoint.x,globalPoint.y,theta1;//当前的
            ekfilter.update(z);

            //预估完成
            Eigen::Vector3d xt = ekfilter.getState();
            globalPoint.x = xt(0);
            globalPoint.y = xt(1);
        }
#endif
        
        //ODOMETRY FUNCTIONS
        void updateSensors(){

            // 读取当前horizonal方向里程计的总转周（inches）
            encoderVal.horizonal = OMath.degToInch(encoderHorizonal.rotation(deg)); //horizonalE
            encoderVal.vertical = OMath.degToInch(encoderVertical.rotation(deg)); //horizonalE

            // 计算编码器变化的delta
            deltaEncoderVal.vertical = encoderVal.vertical - prevEncoderVal.vertical; //verticalE
            deltaEncoderVal.horizonal = encoderVal.horizonal - prevEncoderVal.horizonal; //horizonalE

            // 更新
            prevEncoderVal.vertical = encoderVal.vertical; //verticalE
            prevEncoderVal.horizonal = encoderVal.horizonal; //horizonalE

            // 获取当前朝向
            currentAngle = OMath.getRadians(imu.rotation());
            currentAngle = Math::getWrap2pi(currentAngle);

            deltaAngle = currentAngle - prevAngle;
            prevAngle = currentAngle;

        }

        void updatePosition(){
            //Polar coordinates
            T deltaX = deltaEncoderVal.horizonal;
            T deltaY = deltaEncoderVal.vertical;
            T localX = 0;
            T localY = 0;

            if (deltaAngle == 0){ // prevent divide by 0
                localX = deltaX;
                localY = deltaY;
            }
            else{    
                localX = 2 * sin(deltaAngle / 2) * (deltaX / deltaAngle + horizonalOffset);
                localY = 2 * sin(deltaAngle / 2) * (deltaY / deltaAngle + verticalOffset);
            }
            T  global_angle = prevAngle + deltaAngle/2 - PI * rotate_degree / 180;

            //Cartesian coordinates
            globalDeltaPoint.x = (localY * sin(global_angle)) + (localX * cos(global_angle)); 
            globalDeltaPoint.y = -((localY * cos(global_angle)) - (localX * sin(global_angle)));
            globalDeltaPoint.angle = deltaAngle;

            globalPoint.x = globalDeltaPoint.x + prevGlobalPoint.x;
            globalPoint.y = globalDeltaPoint.y + prevGlobalPoint.y;
            globalPoint.angle = currentAngle;

            prevGlobalPoint.x = globalPoint.x;
            prevGlobalPoint.y = globalPoint.y;
    
            #ifdef KalmanFilter
            filteringData();
            #endif



            return;
        }

        //将机器人的位置和传感器值重置为初始状态
        void reset(){
            encoderVertical.resetRotation(); 
            encoderHorizonal.resetRotation(); 
            prevEncoderVal.vertical = 0.0; prevEncoderVal.horizonal = 0.0; 
            prevAngle = 0.0;
            prevGlobalPoint.x = 0.0; prevGlobalPoint.y = 0.0;
        }

        void setPosition(float newX, float newY, float newAngle) override{
            //reset();
            prevAngle = newAngle;
            prevGlobalPoint.x = newX;
            prevGlobalPoint.y = newY;
            globalPoint.angle = newAngle;
            globalPoint.x = newX;
            globalPoint.y = newY;
            imu.setRotation(newAngle, deg);
            #ifdef KalmanFilter
            ekfilter.setx0(newX, newY);
            #endif

        }

        //ODOMETRY THREAD
        void OdomRun(Odom& odom){
            T LeftSideMotorPosition=0, RightSideMotorPosition = 0;
            T LastLeftSideMotorPosition=0, LastRightSideMotorPosition = 0;
            while(true) { 
                // 更新电机编码器
                for(motor* m : _leftMotors) LeftSideMotorPosition += OMath.getRadians(m->position(deg))/_leftMotors.size();
                for(motor* m : _rightMotors) RightSideMotorPosition += OMath.getRadians(m->position(deg))/_rightMotors.size();
                T LeftSideDistance=(LeftSideMotorPosition - LastLeftSideMotorPosition)*r_motor; 
                T RightSideDistance=(RightSideMotorPosition-LastRightSideMotorPosition)*r_motor;
                LeftBaseDistance = LeftSideMotorPosition*r_motor; RightBaseDistance = RightSideMotorPosition*r_motor;
                // 更新轮式里程计
                odom.updateSensors();
                // 更新位置坐标
                odom.updatePosition();
                printf("X: %lf, Y: %lf\n",odom.globalPoint.x / cell,odom.globalPoint.y / cell);
                this_thread::sleep_for(10); 
            }
        }

        void executePosition() override{
            OdomRun(*this);
        }

    };
};

