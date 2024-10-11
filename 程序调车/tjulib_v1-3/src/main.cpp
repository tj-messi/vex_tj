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

using namespace vex;
using namespace tjulib;

/*---------------  模式选择  ---------------*/
// 如果进行技能赛就def，否则注释，进行自动
//#define SKILL
// 如果用里程计就def，否则注释，用雷达
#define ODOM
// 如果要开启远程调试就def，否则就注释
#define Remotedeubug
/**************************电机定义***********************************/
// ordinary chassis define
//std::vector<std::vector<vex::motor*>*> _chassisMotors = { &_leftMotors, &_rightMotors} ;
// oct chassis define
std::vector<std::vector<vex::motor*>*> _chassisMotors = {&_lfMotors, &_lbMotors, &_rfMotors, &_rbMotors};
/**************************调参区域***********************************/

// Definition of const variables
//const double PI = 3.1415926;

// imu零漂误差修正
double zero_drift_error = 0;  // 零漂误差修正，程序执行时不断增大
double correct_rate = 0.0000;

// 全局计时器
static timer global_time;  

competition Competition;

/*************************************

        pid configurations

*************************************/

/*configure meanings：
    ki, kp, kd, 
    integral's active zone (either inches or degrees), 
    error's thredhold      (either inches or degrees),
    minSpeed               (in voltage),
    stop_num               (int_type)
*/

pidParams   fwd_pid(8, 0.6, 0.3, 1, 1.2, 20, 8), 
            turn_pid(3, 0.05, 0.05, 3, 1, 10, 15), 
            cur_pid(8.0, 0.05, 0.15, 3, 1, 20, 15),
            straightline_pid(2.8, 0.1, 0.12, 5, 1, 1, 0),
            wheelmotor_pid(0.25, 0.01, 0.02, 50, 5, 0, 10);

/*************************************

        Instance for position

*************************************/
//Dif_Odom diff_odom(_leftMotors, _rightMotors,  PI * r_motor * 2, r_motor, imu);
// gps correction
tjulib::GPS gps_(GPS_, gps_offset_x, gps_offset_y);
// odom(of 45 degree) strategy
Odom odom(hOffset, vOffset, r_wheel_encoder, encoder_rotate_degree, encoderVertical, encoderHorizonal, imu);
// diff-odom strategy ----- diff_odom default
Context *PosTrack = new Context(&odom); 
// vector for all position strategy
std::vector<Position*>_PositionStrategy = {&odom};
/*************************************

        Instance for control

*************************************/
// Declaration of PID parameters and PID controllers
pidControl curControl(&cur_pid);
pidControl fwdControl(&fwd_pid);
pidControl turnControl(&turn_pid);
pidControl straightlineControl(&straightline_pid);
pidControl motorControl(&wheelmotor_pid);
// 底盘控制
//Ordi_SmartChassis FDrive(_chassisMotors, &motorControl, PosTrack->position, r_motor, &curControl, &fwdControl, &turnControl, car_width);
Oct_SmartChassis ODrive(_chassisMotors, &motorControl, PosTrack->position, r_motor, &curControl, &fwdControl, &turnControl, car_width);
// 导入杆
Load loading(&side_bar);
// 远程调试
RemoteDebug remotedebug(PosTrack->position); 
/***************************
 
      thread define

 **************************/

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
// PosTrack 定位线程，在这里选择定位策略
int PositionTrack(){

    // _PositionStrategy has {&diff_odom, &odom}
    PosTrack = new Context(_PositionStrategy[0]);
    PosTrack->startPosition();
    return 0;

}

int GPS_update(){
    
    timer time;
    time.clear();
    int flag = 1;
    while(1){
       
        imu.setHeading(GPS_.heading(deg), deg);

        gps_x = gps_.gpsX();
        gps_y = gps_.gpsY();
        gps_heading = gps_.gpsHeading();
        
        if((time.time(msec)-3000)<=10 && flag){
            imu.setHeading(GPS_.heading(deg), deg);
            // 第4秒的时候会更新一下坐标
            PosTrack->setPosition({gps_x, gps_y, GPS_.heading(deg) / 180 * 3.14159});
            
            printf("position initialization finish\n");

            flag = 0;
        }
        task::sleep(10);
        
    }
        
        
}
/***************************
 
    pre-autonomous run

 **************************/
// 设置初始位置、角度
#ifdef SKILL
    // 初始位置，单位为inches
    double init_pos_x = -59;
    double init_pos_y = 35.4;

    // 逆时针角度，范围在0 ~ 360°之间
    double initangle = 0;

#else
    // 初始位置，单位为inches
    double init_pos_x = 0;
    double init_pos_y = 0;

    // 逆时针角度，范围在0 ~ 360°之间
    double init_angle = 0;

#endif
void pre_auton(){
    thread PosTrack_(PositionTrack);
/***********是否开启远程调试************/
#ifdef Remotedeubug
    thread Remotedebug(RemoteDubug);
#endif
/***********imu、gps、distancesensor、vision等设备初始化************/  
    
    printf("pre-auton start\n");
    if(GPS_.installed()){
        GPS_.calibrate();
        while(GPS_.isCalibrating()) task::sleep(8);
        
    }

    // 这里考虑到只使用imu而不使用gps的情况
    if(imu.installed()){
        // 设置初始位置
        PosTrack->setPosition({init_pos_x, init_pos_y, init_angle});
    }
    
    if(GPS_.installed()){
        thread GPS_update_(GPS_update);
    }

    loading.setPosition(RIGHT);

    

    printf("pre-auton finish\n");
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
    sendTask();  // 向联队车发送信息
    task::sleep(200);
    Brain.Screen.print("send thread jump out\n");

    /************************
      
      发送完信号后执行的程序
      
    ************************/

    // 等待一下
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
    // ODrive.turnToAngle(270, 100, 5000, 1);
    ODrive.moveInches(1 * cell, 90, 60,3000, 1);

}

/***************************
 
      skillautonomous run

 **************************/
void skillautonoumous(){
   
}
/***************************
 
      usercontrol run

 **************************/
void Run(double l, double r) {
    L1.spin(fwd, l, pct);
    L2.spin(fwd, l, pct);
    L3.spin(fwd, l, pct);
    L4.spin(fwd, l, pct);
    R1.spin(fwd, r, pct);
    R2.spin(fwd,r, pct);
    R3.spin(fwd,r, pct);
    R4.spin(fwd, r, pct);
}

void usercontrol()
{

        while(1)
    {

        int fb,lf;

        fb=Controller1.Axis3.value();
        lf=Controller1.Axis4.value();
        fb=std::abs(fb)>15?fb:0;
        lf=std::abs(lf)>15?lf:0;
        if(fb!=0||lf!=0) Run((fb+lf)*100.0/127.0,(fb-lf)*100.0/127.0);
        else Run(0,0);
        //std::Sleep(8);//注意要sleep一小段时间防止过载
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

