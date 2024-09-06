/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Author:       TJU-CodeWeavers                                           */
/*    Created:      2023/11/1 23:12:20                                        */
/*    Description:  tjulib for V5 project                                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "tjulib.h"
#include <string>
/*---------------  模式选择  ---------------*/
// 如果进�?�技能赛就def，否则注释，进�?�自�?
//#define SKILL
// 如果用里程�?�就def，否则注释，用雷�?
#define ODOM
// 如果要开�?远程调试就def，否则就注释
#define Remotedeubug


using namespace vex;
using namespace tjulib;
/**************************调参区域***********************************/

// Definition of const variables
const double PI = 3.1415926;

// imu零漂�?�?�?�?
double zero_drift_error = 0;  // 零漂�?�?�?正，程序执�?�时不断增大
double correct_rate = 0.0000;

// 全局计时�?
static timer global_time;  

competition Competition;

/*************************************

        pid configurations

*************************************/

/*configure meanings�?
    ki, kp, kd, 
    integral's active zone (either inches or degrees), 
    error's thredhold      (either inches or degrees),
    minSpeed               (in voltage),
    stop_num               (int_type)
*/

pidParams   fwd_pid(8, 0.6, 0.3, 1, 0.3, 20, 8), 
            turn_pid(1.5, 0.05, 0.05, 3, 1, 20, 10), 
            cur_pid(8.0, 0.05, 0.15, 3, 1, 20, 15),
            straightline_pid(2.8, 0.1, 0.12, 5, 1, 1, 0);


/*************************************

        Instance for position

*************************************/

Dif_Odom diff_odom(_leftMotors, _rightMotors,  PI * r_motor * 2, r_motor, imu);
// odom(of 45 degree) strategy
Odom odom(hOffset, vOffset,  encoderCircumference, 45, encoderVertical, encoderHorizonal, imu);
// diff-odom strategy ----- diff_odom default
Context *PosTrack = new Context(&diff_odom); 
// vector for all position strategy
std::vector<Position*>_PositionStrategy = {&diff_odom, &odom};
/*************************************

        Instance for control

*************************************/
// Declaration of PID parameters and PID controllers
pidControl curControl(&cur_pid);
pidControl fwdControl(&fwd_pid);
pidControl turnControl(&turn_pid);
pidControl straightlineControl(&straightline_pid);
// 底盘控制
SmartChassis Drive(_leftMotors, _rightMotors, PosTrack->position, &curControl, &fwdControl, &turnControl, &straightlineControl, car_width);
// 导入�?
Load loading(&side_bar);
// 远程调试
RemoteDebug remotedebug(PosTrack->position); 
/***************************
 
      thread define

 **************************/
// PosTrack 定位线程，在这里选择定位策略
int PositionTrack(){

    // _PositionStrategy has {&diff_odom, &odom}
    PosTrack = new Context(_PositionStrategy[0]);
    PosTrack->startPosition();

    return 0;

}
// 远程调试
int RemoteDubug(){

#ifdef DashBoard
    remotedebug.PositionDebugSerial();
#else

#endif
    return 0;
}
/***************************
 
      initial pos set

 **************************/

/***************************
 
    pre-autonomous run

 **************************/
// 设置初�?�位�?、�?�度
#ifdef SKILL
    // 初�?�位�?，单位为inches
    double init_pos_x = -59;
    double init_pos_y = 35.4;

    // 逆时针�?�度，范围在0 ~ 360°之间
    double initangle = 0;

#else
    // 初�?�位�?，单位为inches
    double init_pos_x = 0 * cell;
    double init_pos_y = 0 * cell;

    // 逆时针�?�度，范围在0 ~ 360°之间
    double init_angle = 0;

#endif
void pre_auton(){
    thread PosTrack_(PositionTrack);
/***********�?否开�?远程调试************/
#ifdef Remotedeubug
    thread Remotedebug(RemoteDubug);
#endif
/***********imu、gps、distancesensor、vision等�?��?�初始化************/  
    
    
    if(imu.installed()){
        imu.calibrate();
        while(imu.isCalibrating()) task::sleep(8);
    }
        
    if(GPS.installed()){
        GPS.calibrate();
        while(GPS.isCalibrating()) task::sleep(8);
    }
    
    // 设置初�?�位�?
    PosTrack->setPosition({init_pos_x, init_pos_y, init_angle});
    GPS.setRotation(init_angle, deg);
    GPS.setOrigin(init_pos_x, init_pos_y, inches);
   
    loading.setPosition(RIGHT);
    
    task::sleep(3000);
}

/*********************************
 
    Dual-Communication Thread

 ***********************************/
static int received_flag = 0;
int sendTask(){

    while( !AllianceLink.isLinked() )
        this_thread::sleep_for(8);

    AllianceLink.send("run");
    Brain.Screen.print("successfully sended\n");
    return 0;
}
void confirm_SmallCar_Finished(const char* message, const char*linkname, double nums){

    received_flag = 1;
    Brain.Screen.print("successfully received\n");
}    
// Dual-Communication Demo
void demo_dualCommunication(){
    sendTask();  // 向联队车发送信�?
    task::sleep(200);
    Brain.Screen.print("send thread jump out\n");

    /************************
      
      发送完信号后执行的程序
      
    ************************/

    // 等待一�?
    while(1){
        AllianceLink.received("finished", confirm_SmallCar_Finished);
        task::sleep(200);
        if(received_flag){break;}
    }

}

/***************************
 
      autonomous run

 **************************/
void autonomous(){
    //Drive.turnToAngle(-90, 80, 1000, 1);
    Drive.moveInches(1 * cell, 100, 5000, 1);

    Drive.moveInches(2 * cell, 100, 5000, 1);


    
   // Drive.CurPIDMove({2*cell, 2*cell, 0}, 80, 8000,1);
}

/***************************
 
      skillautonomous run

 **************************/
void skillautonoumous(){
   
}
/***************************
 
      usercontrol run

 **************************/
void usercontrol()
{
    Controller1.ButtonL1.pressed([]() {
        lift_arm.spin(forward); // 电机正转
    });

    Controller1.ButtonL1.released([]() {
        lift_arm.setStopping(brakeType::hold);
        lift_arm.stop(hold);
    });

    Controller1.ButtonL2.pressed([]() {
         lift_arm.spin(vex::reverse); // 电机正转
    });

    Controller1.ButtonL2.released([]() {
        lift_arm.setStopping(brakeType::hold);
        lift_arm.stop(hold);
    });
    Controller1.ButtonR1.pressed([]() {
        static bool motorRunning = false; // 用于追踪电机状�?

        if (!motorRunning) {
            roller_group.spin(forward,100,pct);
        } else {
           roller_group.stop();// 停�?�电机旋�?
        }
        motorRunning = !motorRunning; // 切换电机状态}
    });

    Controller1.ButtonR2.pressed([]() {
        static bool motorRunning = false; // 用于追踪电机状�?

        if (!motorRunning) {
            roller_group.spin(forward,-100,pct);
        } else {
           roller_group.stop();// 停�?�电机旋�?
        }
        motorRunning = !motorRunning; // 切换电机状态}
    });


    Controller1.ButtonL1.released([]() {
        lift_arm.setStopping(brakeType::hold);
        lift_arm.stop(hold);
    });

     Controller1.ButtonA.pressed([]() {
         static bool status = false; // 用于追踪电机状�?

         if (!status) {
             gas1.state(100,pct);
         } else {
             gas1.state(0,pct);
         }
         status = !status; // 切换状�?
     });

    Controller1.ButtonB.pressed([]() {
         static bool status = false; // 用于追踪电机状�?

         if (!status) {
             gas2.state(1200,pct);
         } else {
             gas2.state(0,pct);
         }
         status = !status; // 切换状�?
     });

    while(true){
        Drive.ArcadeDrive();

        // 调试时通过按键进入�?�?
         if(Controller1.ButtonX.pressing()){
             autonomous();
         }
         if(Controller1.ButtonY.pressing()){
             skillautonoumous();
         }

        if(Controller1.ButtonUp.pressing()){
            vexMotorVoltageSet(side_bar.index(), 100*120);
        }else if(Controller1.ButtonDown.pressing()){
            vexMotorVoltageSet(side_bar.index(), -100*120);
        }else{
            side_bar.stop(hold);
        }

    }
}


int main() {
  // Set up callbacks for autonomous and driver control periods.
  #ifdef SKILL
  Competition.autonomous(skillautonoumous);
  #else

    Competition.autonomous(autonomous);

  #endif


  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

