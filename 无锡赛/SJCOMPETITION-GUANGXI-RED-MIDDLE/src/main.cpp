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
// 如果进�?�技能赛就def，否则注释，进�?�自�?
// #define SKILL
// 如果用里程�?�就def，否则注释，用雷�?
#define ODOM
// 如果要开�?远程调试就def，否则就注释
#define Remotedeubug
// 如果�?红方就def，否则就注释
// #define RED

/**************************电机定义***********************************/
// ordinary chassis define
std::vector<std::vector<vex::motor *> *> _chassisMotors = {&_leftMotors, &_rightMotors};
// oct chassis define
// std::vector<std::vector<vex::motor *> *> _chassisMotors = {&_lfMotors, &_lbMotors, &_rfMotors, &_rbMotors};
/**************************调参区域***********************************/

// Definition of const variables
const double PI = 3.1415926535;

// imu零漂�?�?�?�?
double zero_drift_error = 0; // 零漂�?�?�?正，程序执�?�时不断增大
double correct_rate = 0.0000;

// 全局计时�?
static timer global_time;
// 竞赛模板�?
competition Competition;
// vex-ai jeson nano comms
ai::jetson jetson_comms;

pwm_out gas;
// 红方标志
#ifdef RED
bool is_red = true;
#else
bool is_red = false;
#endif
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

pidParams fwd_pid(5, 0.05, 16, 1, 0.5, 10, 15),
    turn_pid(1.3, 0.081, 0.09, 15, 1, 20, 15),
    cur_pid(8.0, 0.05, 0.15, 3, 1, 20, 15),
    straightline_pid(10, 0.1, 0.12, 5, 4, 1, 10),
    wheelmotor_pid(0.25, 0.01, 0.02, 50, 5, 0, 10);

// pidParams fwd_pid(6.5, 0.35, 0.3, 2, 2.5, 7, 15),
//     turn_pid(2, 0.15, 0.35, 45, 1, 5, 15),
//     cur_pid(8.0, 0.05, 0.15, 3, 1, 20, 15),
//     straightline_pid(10, 0.1, 0.12, 5, 4, 1, 10),
//     wheelmotor_pid(0.25, 0.01, 0.02, 50, 5, 0, 10);

/*************************************

        Instance for position

*************************************/
Dif_Odom diff_odom(_leftMotors, _rightMotors, PI *r_motor * 2, r_motor, imu);
//
// gps correction
tjulib::GPS gps_(GPS_, gps_offset_x, gps_offset_y);
// odom(of 45 degree) strategy
// Odom odom(hOffset, vOffset, r_wheel_encoder, encoder_rotate_degree, encoderVertical, encoderHorizonal, imu);
// diff-odom strategy ----- diff_odom default
Context *PosTrack = new Context(&diff_odom);
// vector for all position strategy
std::vector<Position *> _PositionStrategy = {&diff_odom};

/*************************************

        Instance for map

*************************************/
// local storage for latest data from the Jetson Nano
AI_RECORD local_map;
HighStakeMap map(PosTrack->position);

/*************************************

        Instance for control

*************************************/

// ====Declaration of PID parameters and PID controllers ====
pidControl curControl(&cur_pid);
pidControl fwdControl(&fwd_pid);
pidControl turnControl(&turn_pid);
pidControl straightlineControl(&straightline_pid);
pidControl motorControl(&wheelmotor_pid);

// ====Declaration of Path Planner and Controller ====
// Declaration of rrt planner
RRT rrtPlanner_short(map.obstacleList, -72, 72, 2, 25, 20000, 4);
RRT rrtPlanner_long(map.obstacleList, -72, 72, 3, 20, 20000, 12);
// Declaration of PurPursuit Controller
PurePursuit purepursuitControl(PosTrack->position);

// ====Declaration of Chassis Controller ====
// 底盘控制
Ordi_SmartChassis FDrive(_chassisMotors, &motorControl, PosTrack->position, r_motor, &curControl, &fwdControl, &turnControl, car_width);
// Oct_SmartChassis ODrive(_chassisMotors, &motorControl, PosTrack->position, r_motor, &curControl, &fwdControl, &turnControl, car_width, &purepursuitControl, &map, &rrtPlanner_short, &rrtPlanner_long);

/*********************************

    ring_detect Thread

 ***********************************/
/*
{
  "brightness": 50,
  "signatures": [
    {
      "name": "SIG_1",
      "parameters": {
        "uMin": -4063,
        "uMax": -3125,
        "uMean": -3594,
        "vMin": 3633,
        "vMax": 5729,
        "vMean": 4681,
        "rgb": 4218760,
        "type": 0,
        "name": "SIG_1"
      },
      "range": 2.5
    },
    BLUE
    vision::signature(1,-4063,-3125,-3594,3633,5729,4681,2.5,0);
    {
      "name": "SIG_2",
      "parameters": {
        "uMin": 9403,
        "uMax": 11891,
        "uMean": 10647,
        "vMin": -1903,
        "vMax": -1413,
        "vMean": -1658,
        "rgb": 11159866,
        "type": 0,
        "name": "SIG_2"
      },
      "range": 2.5
    },
    RED
    vision::signature(2,9403,11891,10647,-1903,-1413,-1658,2.5,0)
}
*/
enum RING_STATE
{
    NORING,
    RING_COMING_IN,
    RING_GOING_OUT
};
#ifdef RED
vision::signature our_ring = vision::signature(2, 9403, 11891, 10647, -1903, -1413, -1658, 2.5, 0);
vision::signature enemy_ring = vision::signature(1, -4063, -3125, -3594, 3633, 5729, 4681, 2.5, 0);
#else
vision::signature our_ring = vision::signature(1, -4063, -3125, -3594, 3633, 5729, 4681, 2.5, 0);
vision::signature enemy_ring = vision::signature(2, 9403, 11891, 10647, -1903, -1413, -1658, 2.5, 0);
#endif
vision Vision1 = vision(PORT2, 50, our_ring, enemy_ring);

int ring_detect()
{
    // int ring_state = NORING;
    // int high_threshold = 200;
    // int low_threshold = 110;
    while (1)
    {
        Vision1.takeSnapshot(enemy_ring);
        // if (Vision.largestObject.width > high_threshold && ring_state == NORING)
        // {
        //     ring_state = RING_COMING_IN;
        // }
        // else if (Vision.largestObject.width < low_threshold && ring_state == RING_COMING_IN)
        // {
        //     ring_state = RING_GOING_OUT;
        // }
        // else if (Vision.largestObject.width < low_threshold && ring_state == RING_GOING_OUT)
        // {
        //     ring_state = NORING;
        // }
        if (Vision1.largestObject.width > 250 && convey_belt.direction() == forward)
        {
            if (is_red)
                wait(125, msec);
            else
                wait(120, msec);
            convey_belt.stop();
            wait(250, msec);
            convey_belt.spin(forward, 100, pct);
        }
        wait(4, msec);
    }
}
// thread ring_detect_(ring_detect);

/***************************

      thread define

 **************************/
// 远程调试
RemoteDebug remotedebug(PosTrack->position);
// 远程调试
int RemoteDubug()
{

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
int PositionTrack()
{

    // _PositionStrategy has {&diff_odom, &odom}
    PosTrack = new Context(_PositionStrategy[0]);
    PosTrack->startPosition();
    return 0;
}
void Get_SmallCar_GPS(const char *message, const char *linkname, double nums)
{

    sscanf(message, "%lf,%lf", &gps_x_small, &gps_y_small);
    printf("%lf,%lf\n", gps_x_small, gps_y_small);

    Brain.Screen.print("successfully received\n");
}
int receivedTask()
{

    while (!AllianceLink.isLinked())
        this_thread::sleep_for(8);

    gps_x_small = 0;
    gps_y_small = 0;
    while (1)
    {
        AllianceLink.received(Get_SmallCar_GPS);
        task::sleep(20);
    }
    return 0;
}

// 更新线程
int GPS_update()
{

    timer time;
    time.clear();
    int flag = 1;
    imu.setHeading(GPS_.heading(deg), deg);
    while (1)
    {

        // imu.setHeading(GPS_.heading(deg), deg);

        gps_x = gps_.gpsX();
        gps_y = gps_.gpsY();
        gps_heading = GPS_.heading(deg);

        if ((time.time(msec) - 3000) <= 50 && flag)
        {
            imu.setHeading(GPS_.heading(deg), deg);
            imu.setRotation(GPS_.heading(deg), deg);
            // �?4秒的时候会更新一下坐�?
            PosTrack->setPosition({gps_x, gps_y, GPS_.heading(deg) / 180 * 3.1415926535});

            printf("position initialization finish\n");

            flag = 0;
        }
        task::sleep(10);
    }
}
void VisionTest()
{
    while (1)
    {
        Vision_front.takeSnapshot(Stake_Red);
        if (Vision_front.largestObject.exists)
        {
            int x = Vision_front.largestObject.centerX;
            printf("Red : center_x : %d\n", x);
        }
        Vision_front.takeSnapshot(Stake_Blue);
        if (Vision_front.largestObject.exists)
        {
            int x = Vision_front.largestObject.centerX;
            printf("Blue : center_x : %d\n", x);
        }
        Vision_front.takeSnapshot(Stake_Yellow);
        if (Vision_front.largestObject.exists)
        {
            int x = Vision_front.largestObject.centerX;
            printf("Yellow : center_x : %d\n", x);
        }
        task::sleep(30);
    }
}
/***************************

    pre-autonomous run

 **************************/
// 设置初�?�位�?、�?�度
#ifdef SKILL
// 初�?�位�?，单位为inches
double init_pos_x = -59;
double init_pos_y = 35.4;

// 逆时针�?�度，范围在0 ~ 360°之间
double init_angle = 0;

#else
// 初�?�位�?，单位为inches
double init_pos_x = 0;
double init_pos_y = 0;

// 逆时针�?�度，范围在0 ~ 360°之间
double init_angle = 0;

#endif
void pre_auton()
{
#ifdef RED
    is_red = true;
#else
    is_red = false;
#endif
    thread PosTrack_(PositionTrack);
/***********�?否开�?远程调试************/
#ifdef Remotedeubug
    thread Remotedebug(RemoteDubug);
#endif
    /***********imu、gps、distancesensor、vision等�?��?�初始化************/

    printf("pre-auton start\n");
    if (GPS_.installed())
    {
        GPS_.calibrate();
        while (GPS_.isCalibrating())
            task::sleep(8);
    }
    thread receive(receivedTask);

    // 这里考虑到只使用imu而不使用gps的情�?
    if (imu.installed())
    {
        // 设置初�?�位�?
        PosTrack->setPosition({init_pos_x, init_pos_y, init_angle});
    }
    // GPS更新线程
    if (GPS_.installed())
    {
        thread GPS_update_(GPS_update);
    }
    // thread testvision(VisionTest);
    //  吸环线程
    // thread GetRing_(GetRing);
    // thread CheckStuck_(CheckStuck);
    // thread CheckRing_(CheckRing);
    thread ring_detect_(ring_detect);
    lift_arm.spin(reverse, 30, pct); // 电机反转
    wait(0.2, seconds);
    lift_arm.spin(forward, 0, pct);
    wait(1, seconds);
    ArmRotationSensor.resetPosition();
    roller_group_rotation.resetPosition();
    printf("pre-auton finish\n");
    task::sleep(300);
}

/*********************************

    Dual-Communication Thread

 ***********************************/
static int received_flag = 0;

// Dual-Communication Demo

int push_enemyring_thread()
{
    task::sleep(2500);
    gas_push.state(100, pct);
    task::sleep(1000);
    gas_push.state(0, pct);
    return 0;
}
int push_enemyring_thread2()
{
    task::sleep(5000);
    gas_push.state(100, pct);
    task::sleep(2000);
    gas_push.state(0, pct);
    return 0;
}
int push_enemyring_thread3()
{
    gas_push.state(100, pct);
    task::sleep(1600);
    gas_push.state(0, pct);
    return 0;
}

int go_out_roller()
{
    while (1)
    {
        manual = true;
        reinforce_stop = true;
        // 当进入桩内的时候退�?
        if ((gps_x * gps_x + gps_y * gps_y) < 24)
        {
            break;
        }
    }

    // 吸一下接着就停
    manual = true;
    ring_convey_spin = true;
    reinforce_stop = false;
    task::sleep(200);
    manual = true;
    reinforce_stop = true;

    // 等到走出来之后就套环
    while (1)
    {
        // 当进入桩内的时候退�?
        if ((gps_x * gps_x + gps_y * gps_y) >= 28)
        {
            manual = true;
            ring_convey_spin = true;
            reinforce_stop = false;
            break;
        }
    }
    return 0;
}

int adjust_roller_position()
{
    roller2clearcorner(1);
    return 0;
}

/***************************

      autonomous run

 **************************/
void clean_corner()
{
    // wait(0, seconds);
    roller2clearcorner(1);
    FDrive.simpleMove(45, 1);
    FDrive.simpleMove(80, 0.5);
    roller_group.spin(forward, 20, pct);
    wait(0.7, seconds);
    FDrive.simpleMove(-12.5, 1);
    // return ;
    convey_belt.spin(forward, 0, pct);
    roller_group.spin(forward, 100, pct);
    convey_belt.spin(forward, 100, pct);
    wait(1, seconds);
    // convey_belt.spin(forward, 100, pct);

    thread adjust_roller_position_1(adjust_roller_position);
    FDrive.turnToAngle(0, 80, 1000);
    wait(1, seconds);
    wait(0.8, seconds);
    FDrive.simpleMove(35, 0.7);
    roller_group.spin(forward, 25, pct);
    wait(0.5, seconds);
    FDrive.simpleMove(-30, 0.5);
    roller_group.spin(forward, 100, pct);
    wait(1, seconds);

    thread adjust_roller_position_2(adjust_roller_position);
    FDrive.turnToAngle(0, 80, 1000);
    wait(1, seconds);
    FDrive.simpleMove(40, 0.8);
    roller_group.spin(forward, 100, pct);
    wait(0.8, seconds);
    FDrive.simpleMove(-40, 0.5);

    FDrive.simpleMove(40, 0.7);
    wait(0.5, seconds);
    FDrive.simpleMove(-40, 0.5);

    FDrive.simpleMove(40, 0.7);
    wait(0.5, seconds);
    FDrive.simpleMove(-40, 0.5);

    convey_belt.spin(forward, 0, pct);
    FDrive.turnToAngle(225, 80, 1500, 1);
    FDrive.simpleMove(-100, 0.3);
    gas_hold.state(0, pct);

    FDrive.simpleMove(100, 0.7);
}

void autonomous()
{
    // 红色方抢夺边�?立桩，新
    // 1.取中间的底座
    // FDrive.moveInches(1.8 * cell, 100, 1300,0);
    // FDrive.turnToAngle(320, 50, 800);
    // FDrive.moveInches(0.38 * cell, 20, 1200,0);
    // wait(0.2, seconds);
    // gas_hold.state(100, pct); 
    // wait(0.3, seconds);
    // // return;

    // // 2.拉回来，放开重新调整
    // FDrive.moveInches(1.4 * cell, 60, 2000);
    // gas_hold.state(0, pct);
    // FDrive.moveInches(0.3 * cell, 60, 1000);
    // // FDrive.turnToAngle(300, 60, 1000);
    // // FDrive.turnToAngle(320, 60, 1000);
    // FDrive.moveInches(0.75 * cell, 60, 1000,0);
    // gas_hold.state(100, pct);

    // // 3.对准另�?�的两个�?
    // FDrive.turnToAngle(270, 70, 2500);
    // // FDrive.turnToAngle(270, 70, 1000);
    // roller_group.spin(forward, 100, pct);
    // convey_belt.spin(forward, 100, pct);
    // FDrive.moveInches(0.9 * cell, 60, 2000);
    // FDrive.turnToAngle(270, 70, 1500);
    // wait(0.5, seconds);
    // FDrive.moveInches(0.8 * cell, 60, 1000);
    // FDrive.moveToTarget(Point{-42,8},80,80,3000,1);
    // wait(0.8, seconds);
    // FDrive.turnToAngle(315, 80, 1500);
    // // return;

    // // 4.清理角落
    // roller2clearcorner(1);
    // FDrive.moveInches(1.5 * cell, 100, 1500, 1);
    // roller_group.spin(forward, 100, pct);
    // wait(1.5, seconds);
    // FDrive.moveInches(0.5 * cell, 100, 1000, 0);  
    // FDrive.turnToAngle(135, 80, 1500);
    // convey_belt.spin(forward, 0, pct);
    // FDrive.moveInches(1 * cell, 100, 500,0);
    // gas_hold.state(0, pct);
    // FDrive.moveInches(1.5 * cell, 100, 1000, 1);
    // FDrive.turnToAngle(0, 80, 1500);
    // FDrive.turnToAngle(0, 80, 1500);
    // FDrive.moveInches(1.2 * cell, 80, 2000);


    // 蓝方镜像程序
    // 1.取中间的底座
    // FDrive.turnToAngle(7, 50, 800);
    FDrive.moveInches(1.8 * cell, 100, 1300,0);
    FDrive.turnToAngle(53, 50, 800);
    FDrive.moveInches(0.46 * cell, 20, 1200,0);
    wait(0.2, seconds);
    gas_hold.state(100, pct);
    wait(0.3, seconds);

    // 2.拉回来，放开重新调整
    FDrive.moveInches(1.4 * cell, 60, 2000);
    gas_hold.state(0, pct);
    FDrive.moveInches(0.3 * cell, 60, 1000);
    FDrive.moveInches(0.75 * cell, 60, 1000,0);
    gas_hold.state(100, pct);

    // 3.对准另�?�的两个�?
    FDrive.turnToAngle(90, 70, 2500);
    roller_group.spin(forward, 100, pct);
    convey_belt.spin(forward, 100, pct);
    FDrive.moveInches(0.9 * cell, 60, 2000);
    FDrive.turnToAngle(90, 70, 1500);
    wait(0.5, seconds);
    FDrive.moveInches(0.8 * cell, 60, 1000);
    // FDrive.moveToTarget(Point{-42,8},80,80,3000,1);
    // wait(0.8, seconds);
    FDrive.turnToAngle(90, 80, 1500);
    FDrive.moveInches(0.5 * cell, 60, 1000,0);
    FDrive.turnToAngle(0, 80, 1500);
    FDrive.moveInches(1.2 * cell, 80, 2000);
    FDrive.turnToAngle(45, 80, 1500);

    // 4.清理角落
    roller2clearcorner(1);
    FDrive.moveInches(1.5 * cell, 100, 1500, 1);
    roller_group.spin(forward, 100, pct);
    wait(1.5, seconds);
    FDrive.moveInches(0.5 * cell, 100, 1000, 0);
    FDrive.turnToAngle(225, 80, 1500);
    convey_belt.spin(forward, 0, pct);
    FDrive.moveInches(1 * cell, 100, 500,0);
    gas_hold.state(0, pct);
    FDrive.moveInches(1.5 * cell, 100, 1000, 1);
    FDrive.turnToAngle(0, 80, 1500);















    /***************************

          红色方抢夺边�?立桩
          小车
          加分�?

     **************************/
    // // 1.取边上的底座
    // FDrive.simpleMove(-100, 0.52);
    // wait(0.98, seconds);
    // gas_hold.state(100, pct);
    // wait(0.2, seconds);
    // // return;

    // // 2.拉回来，放开重新调整
    // FDrive.moveInches(1 * cell, 80, 2000);
    // gas_hold.state(0, pct);
    // FDrive.moveInches(0.3 * cell, 80, 1000);
    // FDrive.moveInches(0.8 * cell, 40, 1500, 0);
    // gas_hold.state(100, pct);

    // // 4.撞�?�重置imu
    // FDrive.turnToAngle(280, 80, 1500);
    // FDrive.moveInches(1 * cell, 60, 1500);
    // imu.resetHeading();

    // // 5.吃掉角落前的一�?�?
    // FDrive.moveInches(0.7 * cell, 100, 1000, 0);
    // FDrive.turnToAngle(90, 80, 1000);
    // roller_group.spin(forward, 100, pct);
    // convey_belt.spin(forward, 100, pct);
    // FDrive.moveInches(1.1 * cell, 60, 2000);
    // FDrive.turnToAngle(45, 80, 1500);
    
    // // 6.清理角落
    // clean_corner();

    /***************************

          红色方抢夺中间中立桩
          小车
          加分�?

     **************************/
    // // 1.取中间的底座
    // FDrive.simpleMove(-100, 0.505);
    // wait(0.85, seconds);
    // gas_hold.state(100, pct);
    // wait(0.2, seconds);
    // // 2.拉回来，放开重新调整
    // FDrive.moveInches(0.5 * cell, 80, 2000);
    // wait(8,sec);
    // FDrive.moveInches(1*cell,80,2000);
    // gas_hold.state(0, pct);
    // FDrive.moveInches(0.3 * cell, 80, 1000);
    // // wait(0.5, seconds);
    // FDrive.moveInches(0.7 * cell, 40, 1500, 0);
    // gas_hold.state(100, pct);
    // FDrive.turnToAngle(325, 80, 1500);


    // // 6.调整牛子位置
    // thread adjust_roller_position_(adjust_roller_position);
    // FDrive.turnToAngle(45, 80, 1500);
    // return;
    // // roller_group.spin(forward, 0, pct);

    // // 7.清理角落
    // convey_belt.spin(forward, 100, pct);
    // clean_corner();
    // // roller_group.stop(hold);

    /***************************

          蓝色方抢夺中间中立桩
          小车
          加分�?

     **************************/
    // // 1.取中间的底座
    // FDrive.simpleMove(-100, 0.52);
    // wait(0.85, seconds);
    // gas_hold.state(100, pct);
    // // return;

    // // 2.拉回来，放开重新调整
    // FDrive.moveInches(0.5 * cell, 80, 2000);
    // wait(8000,msec);
    // FDrive.moveInches(1*cell,80,2000);
    // gas_hold.state(0, pct);
    // FDrive.moveInches(0.3 * cell, 80, 1000);
    // FDrive.moveInches(0.7 * cell, 40, 1500, 0);
    // gas_hold.state(100, pct);

    // // 3.对准另�?�的两个�?
    // FDrive.turnToAngle(74, 80, 1500);
    // convey_belt.spin(forward, 100, pct);
    // roller_group.spin(forward, 100, pct);
    // FDrive.moveInches(1 * cell, 60, 2000);
    // wait(0.5, seconds);

    // FDrive.moveInches(0.8 * cell, 50, 1000);
    // wait(0.5, seconds);
    // // roller_group.spin(forward, 0, pct);
    
    // // 4.撞�?�重�?位置
    // FDrive.moveInches(1.2 * cell, 100, 1500);
    // imu.resetHeading();
    // // return;

    // // 5.吸掉角落前的�?
    // FDrive.moveInches(0.75 * cell, 100, 1000, 0);
    // roller_group.spin(forward, 100, pct);
    // FDrive.turnToAngle(90, 80, 1000);
    // FDrive.moveInches(0.85 * cell, 80, 1500);
    // wait(0.5, seconds);
    // FDrive.moveInches(0.4 * cell, 50, 1000);
    // wait(0.5, seconds);
    // FDrive.moveInches(0.4 * cell, 50, 1000,0);
    // return;

    // // 6.调整牛子位置
    // thread adjust_roller_position_(adjust_roller_position);
    // FDrive.turnToAngle(45, 80, 1500);
    // // roller_group.spin(forward, 0, pct);
    // clean_corner();
}
/***************************

      skillautonomous run

 **************************/
void skillautonoumous()
{
    // 1.联队边桩
    FDrive.moveInches(0.8 * cell, 80, 1300,0);
    FDrive.turnToAngle(45, 80, 2000);
    FDrive.moveInches(1.2 * cell, 80, 1500,0);
    gas_hold.state(100, pct);

    //
    FDrive.turnToAngle(90, 80, 1500);
    roller_group.spin(forward, 100, pct);
    convey_belt.spin(forward, 100, pct);

    FDrive.moveInches(1 * cell, 80, 1500);
    FDrive.turnToAngle(43, 80, 1500);
    FDrive.moveInches(1.4 * cell, 80, 1500);

    FDrive.turnToAngle(270, 80, 1500);
    FDrive.moveInches(1 * cell, 80, 2000);
    FDrive.turnToAngle(265, 80, 1500);
    FDrive.moveInches(1.2 * cell, 80, 1500);
    FDrive.turnToAngle(315, 80, 1500);
    FDrive.moveInches(1.2 * cell, 80, 1500);
    FDrive.moveInches(0.5 * cell, 80, 1500,0);
    FDrive.turnToAngle(135, 80, 1500);
    FDrive.moveInches(0.7 * cell, 80, 1500,0);
    gas_hold.state(0, pct);

    FDrive.moveInches(0.5 * cell, 80, 1500, 1);
    FDrive.turnToAngle(90, 80, 1500);
    convey_belt.spin(forward, 0, pct);
    FDrive.moveInches(2 * cell, 80, 2000,0);
    imu.resetHeading();

    FDrive.moveInches(0.8 * cell, 80, 1500);
    FDrive.turnToAngle(90, 80, 2000);
    FDrive.moveInches(1 * cell, 80, 1500);
    FDrive.turnToAngle(90, 80, 1500);
    FDrive.moveInches(1 * cell, 80, 1500);
    // roller_group.spin(forward, 0, pct);

    FDrive.turnToAngle(205, 80, 1500);
    FDrive.moveInches(1.4 * cell, 80, 1500,0);
    gas_hold.state(100, pct);
    FDrive.turnToAngle(180, 80, 1500);
}
/***************************

      usercontrol run

 **************************/
void usercontrol()
{
    Controller1.ButtonL1.pressed([]()
                                 {
                                     lift_arm.spin(forward, 60, pct); // 电机正转
                                     if(ArmRotationSensor.position(degrees) > 140)
                                     {
                                         lift_arm.spin(forward, 0, pct);
                                     } });

    Controller1.ButtonL1.released([]()
                                  { lift_arm.spin(forward, 0, pct); });

    Controller1.ButtonL2.pressed([]()
                                 {
                                     lift_arm.spin(vex::reverse, 60, pct); // 电机反转
                                     if(ArmRotationSensor.position(degrees) < 0)
                                     {
                                         lift_arm.spin(forward, 0, pct);
                                     } });

    Controller1.ButtonL2.released([]()
                                  { lift_arm.spin(forward, 0, pct); });
    Controller1.ButtonR1.pressed([]()
                                 {
                                     static bool motorRunning = false; // 用于追踪电机状�?

                                     if (!motorRunning)
                                     {
                                         manual = true;
                                         ring_convey_spin = true;
                                         reinforce_stop = false;
                                         roller_group.spin(forward, 100, pct);
                                         convey_belt.spin(forward, 100, pct);
                                     }
                                     else
                                     {
                                         manual = true;
                                         reinforce_stop = true;
                                         roller_group.stop(); // 停�?�电机旋�?
                                         convey_belt.stop();
                                     }
                                     motorRunning = !motorRunning; // 切换电机状态}
                                 });

    Controller1.ButtonR2.pressed([]()
                                 {
                                     static bool motorRunning = false; // 用于追踪电机状�?

                                     if (!motorRunning)
                                     {
                                         //  manual = true;
                                         //  reinforce_stop = false;
                                         convey_belt.spin(forward, 100, pct);
                                         wait(0.3, seconds);
                                         convey_belt.spin(reverse, 60, pct);
                                         wait(0.1, seconds);
                                         convey_belt.spin(forward, 100, pct);
                                         wait(0.3, seconds);
                                         convey_belt.spin(reverse, 60, pct);
                                         wait(0.2, seconds);
                                         convey_belt.stop(hold);
                                         //  roller_group.spin(forward, -100, pct);
                                         //  convey_belt.spin(reverse, 100, pct);
                                     }
                                     else
                                     {
                                         //  manual = true;
                                         //  reinforce_stop = false;
                                         convey_belt.spin(forward, 100, pct);
                                         wait(0.3, seconds);
                                         convey_belt.spin(reverse, 60, pct);
                                         wait(0.1, seconds);
                                         convey_belt.spin(forward, 100, pct);
                                         wait(0.3, seconds);
                                         convey_belt.spin(reverse, 60, pct);
                                         wait(0.2, seconds);
                                         convey_belt.stop(hold);
                                         //  roller_group.stop(); // 停�?�电机旋�?
                                     }
                                     motorRunning = !motorRunning; // 切换电机状�?
                                 });

    // Controller1.ButtonL1.released([]()
    //                               {
    //     lift_arm.setStopping(brakeType::hold);
    //     lift_arm.stop(hold); });

    Controller1.ButtonY.pressed([]()
                                { Arm2getring(); });

    Controller1.ButtonB.pressed([]()
                                {
                                    static bool status_hold = false; // 用于追踪电机状�?

                                    if (!status_hold)
                                    {
                                        gas_hold.state(100, pct);
                                    }
                                    else
                                    {
                                        gas_hold.state(0, pct);
                                    }
                                    status_hold = !status_hold; // 切换状�?
                                });

    Controller1.ButtonDown.pressed([]()
                                   {
                                       static bool motorRunning = false; // 用于追踪电机状�?

                                       if (!motorRunning)
                                       {
                                           // manual = true;
                                           // ring_convey_spin = true;
                                           // reinforce_stop = false;
                                           roller_group.spin(reverse, 100, pct);
                                           convey_belt.spin(reverse, 100, pct);
                                       }
                                       else
                                       {
                                           // manual = true;
                                           // reinforce_stop = true;
                                           roller_group.stop(); // 停�?�电机旋�?
                                           convey_belt.stop();
                                       }
                                       motorRunning = !motorRunning; // 切换电机状�?
                                   });

    while (true)
    {
        FDrive.ManualDrive_nonPID();
        // printf("ArmRotationSensor: %lf \n", ArmRotationSensor.position(degrees));
        // printf("RollerRotationSensor: %lf \n", roller_group_rotation.position(degrees));
        // 调试时通过按键进入�?�?
        if (Controller1.ButtonX.pressing())
        {
            // autonomous();
        }
        if (Controller1.ButtonLeft.pressing())
        {
            // skillautonoumous();
        }

        if (Controller1.ButtonUp.pressing())
        {
            vexMotorVoltageSet(side_bar.index(), 100 * 120);
        }
        else if (Controller1.ButtonDown.pressing())
        {
            vexMotorVoltageSet(side_bar.index(), -100 * 120);
        }
        else
        {
            side_bar.stop(hold);
        }
    }
}

int main()
{

    // local storage for latest data from the Jetson Nano
    static AI_RECORD local_map;

    // Run at about 15Hz
    int32_t loop_time = 33;

    // start the status update display
    thread t1(dashboardTask);

    // print through the controller to the terminal (vexos 1.0.12 is needed)
    // As USB is tied up with Jetson communications we cannot use
    // printf for debug.  If the controller is connected
    // then this can be used as a direct connection to USB on the controller
    // when using VEXcode.
    //
    FILE *fp = fopen("/dev/serial2", "wb");
    this_thread::sleep_for(loop_time);

#ifdef SKILL
    Competition.autonomous(skillautonoumous);
#else

    Competition.autonomous(autonomous);

#endif

    //  Competition.drivercontrol(usercontrol);

    // Run the pre-autonomous function.
    pre_auton();

    // Prevent main from exiting with an infinite loop.
    while (true)
    {
        // get last map data
        jetson_comms.get_data(&local_map);

        // set our location to be sent to partner robot
        link.set_remote_location(local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status);

        // printf("%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az);

        // request new data
        // NOTE: This request should only happen in a single task.
        jetson_comms.request_map();

        // Allow other tasks to run
        this_thread::sleep_for(loop_time);
    }
}
