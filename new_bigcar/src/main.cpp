/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Miracle                                                   */
/*    Created:      2023/11/1 23:12:20                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
// 如果进行技能赛就def，否则注释，进行自动
//#define SKILL true
// 如果用里程计就def，否则注释，用雷达
#define ODOM
//#define SKILL true
#include "vex.h"
#include "tjulib.h"
#include <string>

using namespace vex;
using namespace tjulib;

// 调参区域
/*************************************************************/

// Definition of const variables
const T car_width = 11.30;
const T r_wheel = 2;
const T r_motor = r_wheel * 0.75 ; // /2
const T cell  = 22;
double wheelCircumference = 2.57;

// imu零漂误差修正
double zero_drift_error = 0;  // 零漂误差修正，程序执行时不断增大
double correct_rate = 0.0000;

// 设置初始位置、角度
#ifdef SKILL
    // 初始位置，单位为inches
    double init_pos_x = -59;
    double init_pos_y = 35.4;

    // 逆时针角度，范围在0 ~ 360°之间
    double init_angle = 0;

    int turn_Kp = 2.5;

#else
    // 初始位置，单位为inches
    double init_pos_x = 0;
    double init_pos_y = 0;

    // 逆时针角度，范围在0 ~ 360°之间
    double init_angle = 45;

    int turn_Kp = 1.5;
#endif

// 移动、转向pid，参数可自行调整
pidParams fwd_pid(8.2, 0.05, 0.05, 3, 1, 15), turn_pid(turn_Kp, 0.05, 0.05, 3, 1, 10), cur_pid(8.0, 0.05, 0.05, 3, 1, 15);
/*************************************************************/

//变量定义区
competition Competition;

// Definition of Motor groups for the basement
std::vector<vex::motor*> _leftMotors = {&L1, &L2, &L3,  &L4};
std::vector<vex::motor*> _rightMotors = {&R1, &R2, &R3, &R4};

// Declaration of PID parameters and PID controllers
pidControl difControl(&cur_pid);
pidControl fwdControl(&fwd_pid);
pidControl turnControl(&turn_pid);

// Definition of the tracking and controlling object
Dif_Odom odom(_leftMotors, _rightMotors, wheelCircumference, r_motor, imu);
Dif_Control drive(_leftMotors, _rightMotors, &odom, &difControl, car_width);
HLChassis Hdrive(_leftMotors, _rightMotors, &odom, &difControl, &fwdControl, &turnControl, car_width);
Load loading(&side_bar);

// odom线程
int hodomRun(){
  odom.OdomRun();
  return 0;
}

void pre_auton(){
#ifdef ODOM
    thread posTrack(hodomRun);
   // encoderVertical.resetRotation();
   // encoderHorizonal.resetRotation();

#else
    thread rxThread(receiveTask);  // 启动rx线程
    thread txThread(transmitTask); // 启动tx线程
#endif

    while(!imu.installed())task::sleep(8);
    imu.calibrate();
    while(imu.isCalibrating())task::sleep(8);
    task::sleep(3000);


    odom.setPosition(0, 0, 0);
    loading.setPosition(RIGHT);
}



void kickball(){
    loading.kickLeft(100);
    loading.continuousKick(8);
}

void pushball(){

    Hdrive.turnToAngle(0,65,1000,1);
    Hdrive.setStop(brake);
    Hdrive.moveInches(0.7*cell,80,900,1);
    Hdrive.setStop(brake);
    roller_group.spin(forward,100,pct);
    task::sleep(100);
    Hdrive.moveInches(0.1*cell,80,900,0);
    Hdrive.setStop(brake);

    // 撞墙重置
    Hdrive.turnToAngle(90,65,1500,1);
    Hdrive.setStop(brake);
    task::sleep(50);
    Hdrive.moveInches(0.55*cell,100,900,0);
   // Hdrive.simpleMove(-30,0.50);
    task::sleep(50);
    imu.setRotation(90, deg);
    task::sleep(50);
    // 向前走一步准备掉头推球
    Hdrive.moveInches(0.24*cell,80,900,1);
    Hdrive.setStop(brake);
    task::sleep(50);
    gas1.state(100,pct);
    //gas2.state(100,pct);
    // 小退一步贴近墙壁
    roller_group.spin(forward,0,pct);
    Hdrive.turnToAngle(166,65,1900,1);
    Hdrive.setStop(brake);
    Hdrive.moveInches(0.5*cell,60,3000,0);
    Hdrive.setStop(brake);
    // 回正过洞
    Hdrive.turnToAngle(-176.7,65,1900,1);
    Hdrive.setStop(brake);
    Hdrive.moveInches(2.65*cell,50,3500,0);
    Hdrive.setStop(brake);

}


void taketurn(){

    gas1.state(0,pct);

    // 拨动拐角
    // Hdrive.turnToAngle(-130,90,1000,1);
    // task::sleep(100);
    // Hdrive.turnToAngle(135,65,1300,1);
    // Hdrive.turnToAngle(-130,80,1000,1);
    // task::sleep(100);
    // Hdrive.turnToAngle(135,65,1300,1);

    // 转弯后退
    gas1.state(100,pct);
    gas2.state(100,pct);
    Hdrive.turnToAngle(-170,85,1300,1);
    Hdrive.setStop(brake);
    Hdrive.moveInches(0.57*cell,80,1000,0);
    Hdrive.setStop(brake);
    Hdrive.turnToAngle(-135,85,1300,1);
    Hdrive.setStop(brake);  
    task::sleep(50);
    Hdrive.moveInches(0.65*cell,80,1000,0);
    Hdrive.setStop(brake);

    // 对齐球门准备推
    Hdrive.turnToAngle(-120,85,1200,1);
    Hdrive.simpleMove(-100,0.28);
   // Hdrive.moveInches(0.25*cell,80,1500,0);
    Hdrive.turnToAngle(-90,85,1200,1);
    // Hdrive.moveInches(1.0*cell,80,500,1);
    gas1.state(0,pct);
    gas2.state(0,pct);
    // 向后撞
    Hdrive.simpleMove(35,0.4);
    Hdrive.setStop(brake);
    Hdrive.simpleMove(-100,0.85);
   // imu.setRotation(-135, deg);
    // 向前走
    task::sleep(100);
    Hdrive.simpleMove(40,0.5);
    task::sleep(50);
    Hdrive.simpleMove(-100,0.85); 
    Hdrive.setStop(brake);

    task::sleep(50);
    Hdrive.simpleMove(30,0.53);
    Hdrive.turnToAngle(-135,55,1300,1);
    Hdrive.simpleMove(35,0.37);
    Hdrive.turnToAngle(-90,55,1300,1);
    Hdrive.simpleMove(-100,0.85);
    task::sleep(200);
    
}


void BackToAWP(){

    Hdrive.simpleMove(35,0.32);
    Hdrive.setStop(brake);
    Hdrive.turnToAngle(-135,55,800,1);
    Hdrive.setStop(brake);
    Hdrive.simpleMove(60,0.59);
    Hdrive.setStop(brake);
    task::sleep(50);
    Hdrive.turnToAngle(175,75,800,1);
    Hdrive.setStop(brake);
    Hdrive.simpleMove(60,0.92);
    Hdrive.setStop(brake);

}


void skillresetimu(){

}

void kickColBall(){
    // kick the red ball
    loading.kickLeft(400, 60);
    // three moving to push it
    Hdrive.CurPIDMove({1.0*cell,1*cell,-0}, 30);
    roller_group.spin(forward,-100,pct);
    task::sleep(800);
    loading.kickLeft(500, 60);
    Hdrive.CurPIDMove({0.6*cell,1.2*cell,0}, 40, false);
    Hdrive.turnToAngle(180,30,1000,1);
    Hdrive.simpleMove(-60,0.7);
}

void autonomous(){

    imu.setRotation(0, deg);
    Hdrive.CurPIDMove({0*cell,1*cell,0}, 30);
//    // task::sleep(1000);
//     kickColBall();
//     Hdrive.CurPIDMove({1.1*cell,0.8*cell,-45}, 30);
//     Hdrive.turnToAngle(45, 30, 2000, 1);
    
    

    
//     kickball2(11);
//     side_bar.stop(hold);   

//     task::sleep(8000); 
    
//     pushball2();
//     taketurn2();

//     Hdrive.simpleMove(40, 0.30);
//     Hdrive.setStop(brake);
//     roller_group.spin(forward,+100,pct);

//     Hdrive.moveInches(0.1*cell,80,1000, 0);
}

void skillautonoumous()
{

    
}

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
        static bool motorRunning = false; // 用于追踪电机状态

        if (!motorRunning) {
            roller_group.spin(forward,100,pct);
        } else {
           roller_group.stop();// 停止电机旋转
        }
        motorRunning = !motorRunning; // 切换电机状态}
    });

    Controller1.ButtonR2.pressed([]() {
        static bool motorRunning = false; // 用于追踪电机状态

        if (!motorRunning) {
            roller_group.spin(forward,-100,pct);
        } else {
           roller_group.stop();// 停止电机旋转
        }
        motorRunning = !motorRunning; // 切换电机状态}
    });


    Controller1.ButtonL1.released([]() {
        lift_arm.setStopping(brakeType::hold);
        lift_arm.stop(hold);
    });

     Controller1.ButtonA.pressed([]() {
         static bool status = false; // 用于追踪电机状态

         if (!status) {
             gas1.state(100,pct);
         } else {
             gas1.state(0,pct);
         }
         status = !status; // 切换状态
     });

    Controller1.ButtonB.pressed([]() {
         static bool status = false; // 用于追踪电机状态

         if (!status) {
             gas2.state(1200,pct);
         } else {
             gas2.state(0,pct);
         }
         status = !status; // 切换状态
     });

    while(true){
        drive.ArcadeDrive();
        // double currentAngle = imu.rotation();
        // currentAngle = Math::getWrap360(currentAngle);
        // printf("imu : %lf\n", currentAngle);

        // 调试时通过按键进入自动
        // if(Controller1.ButtonX.pressing()){
        //     autonomous();
        // }
        // if(Controller1.ButtonY.pressing()){
        //     skillautonoumous();
        // }

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

