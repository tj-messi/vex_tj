#pragma once
#include "vex.h"
#include <string>
#include <iostream>
using namespace vex;
typedef double T;

namespace tjulib
{
    // 机械臂重制位置PID控制器函数
    double PIDController_arm(double targetAngle, double currentAngle, double &previousError, double &integral)
    {
        // PID控制器参数
        double Kp = 0.4;   // 比例系数
        double Ki = 0.05; // 积分系数
        double Kd = 4;  // 微分系数
        double error = targetAngle - currentAngle;
        if(fabs(error)<2)
            integral += error;
        double derivative = error - previousError;
        previousError = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }

    int Arm_Lift()
    {
        lift_arm.spin(forward); // 电机正转
        task::sleep(900);
        lift_arm.setStopping(brakeType::hold);
        lift_arm.stop(hold);
        // 放环
        convey_belt.spin(forward, 100, pct);
        return 0;
    }

    int Arm_Down()
    { // arm_lift和arm_down是一起配合使用的
        task::sleep(400);
        convey_belt.spin(forward, 0, pct);
        lift_arm.spin(reverse, 50, pct); // 电机反转
        task::sleep(1000);
        lift_arm.setStopping(brakeType::hold);
        lift_arm.stop(hold);
        return 0;
    }

    void Arm2getring2()
    {
        // lift_arm.spin(reverse, 30, pct); // 电机反转
        // wait(0.4, seconds);
        // ArmRotationSensor.resetPosition();
        // printf("ArmRotationSensor: %lf \n", ArmRotationSensor.position(degrees));
        if(!ArmRotationSensor.installed())
            return;
        double targetAngle = 19; // 目标角度
        double previousError = 0.0;
        double integral = 0.0;
        double currentAngle = ArmRotationSensor.position(degrees);
        int most_times = 40;
        while (most_times--)
        {
            currentAngle = ArmRotationSensor.position(degrees);
            if (fabs(currentAngle - targetAngle) < 0.5)
            {
                int check_times = 5;
                bool pd = true;
                while (check_times--)
                {
                    if (fabs(currentAngle - targetAngle) < 0.5)
                        continue;
                    else
                        pd = false;
                    task::sleep(50);
                    if (!pd)
                        break;
                }
                if (pd)
                    break;
            }
            double error = targetAngle - currentAngle;
            integral += error;
            double derivative = error - previousError;
            previousError = error;

            double speed = PIDController_arm(targetAngle, currentAngle, previousError, integral);
            if (speed > 15)
                speed = 15;
            printf("speed: %lf\n", speed);
            lift_arm.spin(forward, speed, pct);
            wait(0.04, seconds);
            // printf("ArmRotationSensor: %lf \n", ArmRotationSensor.position(degrees));
            // lift_arm.stop(hold);
        }
        lift_arm.stop(hold);
    }

    void Arm2getring()
    {
        // lift_arm.spin(reverse, 30, pct); // 电机反转
        // wait(0.4, seconds);
        // ArmRotationSensor.resetPosition();
        // printf("ArmRotationSensor: %lf \n", ArmRotationSensor.position(degrees));

        double targetAngle = 19; // 目标角度
        double previousError = 0.0;
        double integral = 0.0;
        double currentAngle = ArmRotationSensor.position(degrees);
        int most_times = 40;
        while (most_times--)
        {
            currentAngle = ArmRotationSensor.position(degrees);
            if (fabs(currentAngle - targetAngle) < 0.5)
            {
                int check_times = 5;
                bool pd = true;
                while (check_times--)
                {
                    if (fabs(currentAngle - targetAngle) < 0.5)
                        continue;
                    else
                        pd = false;
                    task::sleep(50);
                    if (!pd)
                        break;
                }
                if (pd)
                    break;
            }
            double error = targetAngle - currentAngle;
            integral += error;
            double derivative = error - previousError;
            previousError = error;

            double speed = PIDController_arm(targetAngle, currentAngle, previousError, integral);
            if (speed > 20)
                speed = 20;
            printf("speed: %lf\n", speed);
            lift_arm.spin(forward, speed, pct);
            wait(0.04, seconds);
            // printf("ArmRotationSensor: %lf \n", ArmRotationSensor.position(degrees));
            // lift_arm.stop(hold);
        }
        if (abs(ArmRotationSensor.position(degrees) - targetAngle) > 0.5)
            Arm2getring2();
        else
            lift_arm.stop(hold);
    }

}