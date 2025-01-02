#pragma once
#include "vex.h"
#include <string>
#include <iostream>
using namespace vex;
typedef double T;

namespace tjulib
{
    timer begin_time;
    // 检查卡环
    int CheckStuck(){
        while(1){
            /*  
                只有在发出吸环信号(转动)的时候才开始检查
                但是由于ring_convey_spin在一开始转动的时候就立马置0了
                所以这里直接使用photo_flag
                                                               */
            if(photo_flag && begin_time.time(msec) > 500 ){
                if(fabs(roller_group.velocity(pct)) < 2 || fabs(convey_belt.velocity(pct)) < 2){
                    ring_convey_stuck = true;
                }
            }
            this_thread::sleep_for(10);
        }
        return 0;
    }

    // 检查环的颜色
    int CheckRing(){
        while(1){
            
            // 如果是没有进入拍照模式，要么是已经刚刚完成一次环套桩，要么就完全没有吸环过
            if(!photo_flag){
                ring_color = 0;        
            }

            if(photo_flag){
               
                // 先拍摄要保留的
                if(is_red){
                    Vision.takeSnapshot(Red1);     
                }else{
                    Vision.takeSnapshot(Blue1);     
                }
                // 排除干扰环，阈值是自己取的，要打印看一看，每辆车、不同的signature是不一样的
                //printf("%d, %d \n", Vision.largestObject.width, Vision.largestObject.height);
                if (Vision.objectCount > 0 && Vision.largestObject.width > 190 && Vision.largestObject.height > 85) {
                    ring_color = 1;     // 要保留
                    //printf("save\n");
                }

                // 再拍摄要丢弃的
                if(is_red){
                    Vision.takeSnapshot(Blue1);     
                }else{
                    Vision.takeSnapshot(Red1);     
                }
                // 排除干扰环，阈值是自己取的，要打印看一看，每辆车、不同的signature是不一样的
                //printf("%d, %d \n", Vision.largestObject.width, Vision.largestObject.height);
                if (Vision.objectCount > 0 && Vision.largestObject.width > 75 && Vision.largestObject.height > 55){ 
                    
                    ring_color = 2;    // 要丢弃
                    //printf("drop\n");
                }
            }
            this_thread::sleep_for(10);
        }
        return 0;
    }

    // 将环扣上去
    int GetRing(){
        
        timer time;
        T whole_time = INT_MIN;
        const T throw_time = 340;   // 要丢弃时候运环的时间
        T save_time = 1000;    // 要保留时候运环的时间 
        if(half_ring_get){
            save_time = 0;
        }
        while(1){
            bool has_seen_ring = false;
            if(ring_convey_spin){       // 接到吸环信号
                ring_convey_spin = false;
                photo_flag = true;
                whole_time = INT_MIN;
                time.clear();
                begin_time.clear();
                roller_group.spin(forward, -100, pct);
                convey_belt.spin(forward, 100, pct);
                has_seen_ring = false;
                
                // 进入抛环的线程
                while(1){
                  // printf("has_seen_ring : %d\n", has_seen_ring);
                    // 首先先要等待环上来，直到看到新的环做出新的反应
                    if(ring_color == 1 && !has_seen_ring){        // 看到要保留的环
                        time.clear();
                        //printf("save\n");
                        whole_time = save_time;
                        convey_belt.setPosition(0, deg);
                        has_seen_ring = true;
                    }else if(ring_color == 2 && !has_seen_ring){   // 看到要丢弃的环
                        time.clear();
                        //printf("throw\n");
                        has_seen_ring = true;
                        convey_belt.setPosition(0, deg);
                        whole_time = throw_time;
                    }else;

                    // 遇到卡环
                    if(ring_convey_stuck){
                        
                        roller_group.spin(reverse, -40, pct);
                        convey_belt.spin(reverse, 40, pct);
                        this_thread::sleep_for(200);
                        roller_group.spin(forward, -100, pct);
                        convey_belt.spin(forward, 100, pct);
                        ring_convey_stuck = false;
                    }
                   
                    // 遇到强制终止
                    if(reinforce_stop){

                        time.clear();
                       
                        roller_group.stop();
                        convey_belt.stop();
                        photo_flag = false;
                        reinforce_stop = false;
                        break;

                    }
                    //printf("%lf \n", convey_belt.position(deg));
                    // 当已经达到既定时间，停止(如果是手动自动进行到下一轮)
                    if(fabs(time.time(msec) - whole_time) < 20 && !reinforce_stop){
                    
                    //if(fabs(convey_belt.position(degrees) - 185) < 6 && !reinforce_stop){
                        
                        time.clear();
                        roller_group.stop();
                        convey_belt.stop();
                        // 准备要丢弃的时候则停顿一下甩出去
                        if(ring_color == 2){
                            this_thread::sleep_for(350);
                        }

                        photo_flag = false;   
                        // 如果是手动，则抛完环之后还要继续进行
                        if(manual){
                            ring_convey_spin = true;
                        }
                        break;
                    }
                }
            }
            this_thread::sleep_for(10);
        }
        return 0;
    }
    
    // 滚筒位置控制PID控制器函数
    double PIDController_roller(double targetAngle, double currentAngle, double &previousError, double &integral)
    {
        // PID控制器参数
        double Kp = 0.2;  // 比例系数
        double Ki = 0.0005;  // 积分系数
        double Kd = 1; // 微分系数
        double error = targetAngle - currentAngle;
        if(fabs(error)<3)
            integral += error;
        double derivative = error - previousError;
        previousError = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }
    int roller2clearcorner(int times)
    {
        // printf("RollerRotationSensor: %lf \n", roller_group_rotation.position(degrees));
        if(times > 3) return 0;
        double targetAngle = 80; // 目标角度
        double previousError = 0.0;
        double integral = 0.0;
        double currentAngle = Math::getWrap360(roller_group_rotation.position(degrees));
        int most_times = 30;
        while (most_times--)
        { 
            currentAngle = Math::getWrap360(roller_group_rotation.position(degrees));
            if(fabs(currentAngle - targetAngle) < 3) 
            {
                int check_times = 5;
                bool pd = true;
                while(check_times--)
                {
                    if(fabs(currentAngle - targetAngle) < 0.5)
                        continue;
                    else pd = false;
                    task::sleep(50);
                    if(!pd) break;
                }
                if(pd) break;
            }
            double error = targetAngle - currentAngle;
            integral += error;
            double derivative = error - previousError;
            previousError = error;

            double speed = PIDController_roller(targetAngle, currentAngle, previousError, integral);
            if(speed > 30) speed = 30;
            roller_group.spin(reverse, speed, pct);
            wait(0.04, seconds); 
            // roller_group.stop(hold);
        }
        roller_group.stop(hold);
        if (abs(Math::getWrap360(roller_group_rotation.position(degrees)) - targetAngle) > 4)
            roller2clearcorner(times++);
        return 0;
    }
}