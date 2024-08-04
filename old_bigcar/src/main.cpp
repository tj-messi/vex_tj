/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       He JinYang?                                                       */
/*    Created:      2024/3/17 20:04:50                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "tjulib.h"

using namespace vex;
using namespace tjulib;

const double PI = 3.14159265358979323846;
competition Competition;

// Definition of const variables
const T car_width = 16.73;
const T r_wheel = 3.25;
const T r_motor = r_wheel / 2; // /2
const T cell  = 24;
const double wheelCircumference = 1.5;   // 2 * r_motor

// Declaration of PID parameters and PID controllers
pidParams cur_pid(3.2, 0.2, 0.1, 3, 4);
pidParams fwd_pid(3.2, 0.2, 0.1, 3, 4);
pidParams turn_pid(1.5, 0.2, 0.1, 3, 1);
pidControl difControl(&cur_pid);
pidControl fwdControl(&fwd_pid);
pidControl turnControl(&turn_pid);
// Definition of Motor groups for the basement
std::vector<vex::motor *> _leftMotors = {&L1, &L2, &L3};
std::vector<vex::motor *> _rightMotors = {&R1, &R2, &R3};
// Definition of the tracking and controlling object
Dif_Odom odom(_leftMotors, _rightMotors, wheelCircumference, r_motor, imu);
Dif_Control drive(_leftMotors, _rightMotors, &odom, &difControl, car_width);
HLChassis Hdrive(_leftMotors, _rightMotors, &odom, &difControl, &fwdControl, &turnControl, car_width);
Load loading(&side_bar);

int hodomRun(){
  odom.OdomRun();
  return 0;
}

void pre_auton()
{
    thread posTracking(hodomRun);
    while (!imu.installed())
        task::sleep(8);
    imu.calibrate();
    while (imu.isCalibrating())
        task::sleep(8);
    task::sleep(3000);

    #ifdef SKILL
        // imu.setRotation(270, deg);
    #else
        odom.setPosition(0.9*cell, 0.9*cell, -45);
    #endif
    loading.setPosition(RIGHT);

    Brain.Screen.printAt( 10, 50, "Preauton Done" );   

}

// Declaration of three main functions
void autonomous();
void skillautonoumous();
void usercontrol();
// Declaration of tool function
void gasopen(){ gas.state(80,pct);}
void gasclose(){ gas.state(0, pct);}
void sideopen(){  side_gas.state(80,pct);}
void sideclose(){ side_gas.state(0, pct);}
void spit(const int V = 100){ cheel.spin(forward,V,pct);}
void intake(const int V = 100){ cheel.spin(reverse,V,pct);}
void intakestop(){ cheel.stop(); }

// Tools used for testing
void stop(){
    while(!Controller1.ButtonX.pressing())
        ;
}
#define $ *cell


void kickColBall(){
    // kick the red ball
    loading.kickLeft(100);

    // three moving to push it
    Hdrive.CurPIDMove({1.4*cell,1*cell,0}, 30);
    loading.kickRight(70);
    Hdrive.CurPIDMove({0.6*cell,1.2*cell,0}, 40, false);
    Hdrive.turnToAngle(180,30,1000,1);
    Hdrive.simpleMove(-60,0.7);
}

int TestMove(){
    kickColBall();
    // make a chaos in the middle
    intake();
    Hdrive.CurPIDMove({1.45*cell,2.18*cell,0}, 40);      // 2.15
    // open gas and push
    sideopen();
    Hdrive.turnToAngle(-80, 50, 2000, 1);
    spit();
    task::sleep(300);    
    Hdrive.turnToAngle(-95, 30, 1000, 1);
    sideclose();
  
    Hdrive.moveInches(1*cell, 20, 2500); 
    task::sleep(400);
    intake();
    Hdrive.moveInches(0.5*cell, 30,2000,false);    
    
    Hdrive.turnToAngle(-120, 30, 1000, 1);    
    spit();
    task::sleep(300);
    
    Hdrive.turnToAngle(-87, 30, 1000, 1);    
    sideopen();
    Hdrive.moveInches(1*cell, 50,2000);  

    sideclose();
    intakestop();

    return 1;
}



int TestBack(){
    // SET POSITION
    odom.setXY(2.6*cell, 2.2*cell);
    // odom.setPosition(2.6*cell, 2.2*cell, -90);
    Hdrive.turnToAngle(-60 ,30, 1000, 1);
    // Hdrive.turnToTarget({1.55*cell,1.4*cell,0},30, 1000, 0);
    Hdrive.CurPIDMove({1.58*cell,1.1*cell,0}, 30, false);
    // Hdrive.moveToTarget({0.7*cell,1.2*cell,0},40,40,3000,0);
    // Hdrive.CurPIDMove({1.5*cell,1.5*cell,0}, 30, false);
    // stop();
    // move back to load balls
    // Hdrive.CurPIDMove({0.95*cell,1.3*cell,0}, 30, false);
    Hdrive.turnToAngle(0, 30, 1000, 1);
    // Hdrive.moveToTarget({odom.globalPoint.x, 1.3*cell,0},20,40,4000,false);
    
    loading.continuousKick(5);
    return 1;

}

int TestMove2(){
    loading.kickLeft(70);
    loading.kickRight(50);

    Hdrive.CurPIDMove({30,25,0}, 25);
    Hdrive.CurPIDMove({18,30,0}, 25, false);
    Hdrive.turnToAngle(180,30,1000,1);
    Hdrive.simpleMove(-35,0.7);
    // make a chaos in the middle
    Hdrive.CurPIDMove({35,36,0}, 40);
    // move back to load balls
    Hdrive.CurPIDMove({25,27,0}, 30, false);
    Hdrive.moveToTarget({25.5,24,0},20,40,4000,false);
    // loading balls
    loading.continuousKick(5);
    stop();
    return 1;
}

int TestGetABall(){
    odom.setPosition(0.95*cell, 0.8*cell,0);
    Hdrive.turnToAngle(-60, 40, 1500, true);
    intake();
    Hdrive.CurPIDMove({1.4*cell, 1.75*cell,0}, 35);
    task::sleep(50);
    intake(50);
    Hdrive.CurPIDMove({1*cell, 0.6*cell,0}, 35, false);
    return 1;

}

// After loading
void CrossMid(){
    // the position after loading is {23,25.7,0}
    odom.setPosition(0.95*cell, 0.8*cell,0);
    // Hdrive.CurPIDMove({0.5*cell, 1.1*cell,0},20);  
    // Hdrive.CurTimeMove(0.05, 20, 1000);
    Hdrive.turnToAngle(45, 30, 1000, 1);

    // Hdrive.CurPIDMove({0.6*cell, 1.4*cell,0},30);
    // {22,35}
    Hdrive.CurPIDMove({1.3*cell, 0.5*cell,0},30,false);
    Hdrive.turnToAngle(84,30,2000,1);
    Hdrive.CurPIDMove({3*cell, 0.5*cell,0},45,false);
    // Hdrive.moveInches(1.9*cell, 50, 3000, false);
}

void CrossMid2(){
    odom.setPosition(0.95*cell, 0.8*cell,0);
    Hdrive.turnToAngle(70, 30, 2000, 1);
    Hdrive.CurPIDMove({3*cell, 0.45*cell,0},45,false);
}

void CrossMid3_base(){
    odom.setXY(0.95*cell, 0.6*cell);
    // Hdrive.turnToAngle(85, 40, 2000, 1);
    Hdrive.turnToAngle(75, 40, 2000, 1);
    Hdrive.CurPIDMove({3*cell, 0.45*cell,0},45,false);
    // Hdrive.CurPIDMove({2.8*cell, 0.55*cell,0},45,false);
}


// Test push balls to the goal
int TestMove3(){
    CrossMid();

    Hdrive.CurPIDMove({3.8*cell, 1.7*cell,0},25,false,3000);
    Hdrive.turnToAngle(180, 30, 1000, 1);
    Hdrive.moveInches(0.5*cell, 30, 1000);
    Hdrive.turnToAngle(160, 30, 1000, 1);
    Hdrive.simpleMove(-70, 1);
    // Hdrive.moveInches(0.9*cell, 70, 2000, 0);

    // move back for awp
    Hdrive.CurPIDMove({2.05*cell, 0.5*cell,0},40);


    // Hdrive.moveInches(2*cell,40,5000,false);
    return 1;
}


int TestMove5(){
    CrossMid();// 70, 11
    Hdrive.moveInches(0.5*cell, 30, 1000);
    Hdrive.CurPIDMove({3.8*cell, 2.3*cell,0},40,false,3000);
    Hdrive.turnToAngle(-90, 30, 1000, false);
    Hdrive.moveInches(0.3*cell, 40, 2000);
    Hdrive.simpleMove(-80, 0.5);
    // move back
    odom.setXY(5.7*cell, 1.6*cell);
    // odom.setPosition(5.7*cell, 1.6*cell, -180);
    Hdrive.turnToAngle(-90, 30, 2000, false);
    Hdrive.CurPIDMove({4.9*cell, 0.8*cell,0},30);
    Hdrive.turnToAngle(-87, 30, 2000, false);
    Hdrive.moveInches(0.4*cell, 40, 2000, false);
    // again
    Hdrive.CurPIDMove({6.5*cell, 4*cell,0},30,false,3000);
    Hdrive.turnToAngle(0, 30, 1000, false);
    Hdrive.moveInches(0.3*cell, 40, 2000);
    task::sleep(600);
    Hdrive.simpleMove(-80, 0.5);

    return 1;
}

void CrossMid3(){
    odom.setXY(0.95*cell, 0.6*cell);
    // Hdrive.turnToAngle(85, 40, 2000, 1);
    Hdrive.turnToAngle(75, 40, 2000, 1);
    // Hdrive.CurPIDMove({2.8*cell, 0.41*cell,0},45,false);
    Hdrive.CurPIDMove({2.8*cell, 0.4*cell,0},45,false);
}


void PushBallsTwice(){
    // Hdrive.moveInches(0.5*cell, 30, 1000);
    Hdrive.turnToAngle(-50, 40, 1000, false); // -60

    Hdrive.CurPIDMove({3.8*cell, 2.3*cell,0},45,false,2200);
    Hdrive.turnToAngle(0, 40, 1000, false);
    spit(30);
    Hdrive.moveInches(0.4*cell, 60, 1000);
    Hdrive.simpleMove(-90, 0.5);
    // move back
    odom.setXY(5.7*cell, 1.6*cell);
    // odom.setPosition(5.7*cell, 1.6*cell, -180);
    spit(50);
    task::sleep(400);

    Hdrive.turnToAngle(-90, 50, 1500, false);
    // Hdrive.turnToAngle(-80, 30, 2000, false);
    intake();

    Hdrive.CurPIDMove({4.9*cell, 0.8*cell,0},30);
    Hdrive.turnToAngle(-87, 50, 1500, false);
    Hdrive.moveInches(0.3*cell, 50, 1500, false);
    // again
    Hdrive.CurPIDMove({5.5*cell, 5*cell,0},50,false,2000);
    Hdrive.turnToAngle(0, 30, 1000, false);
    Hdrive.moveInches(0.4*cell, 40, 1500);

    Hdrive.simpleMove(-90, 0.5);
}

void MoveBackForAWP(const bool head = true){
    odom.setXY(5.7*cell, 1.6*cell);
    Hdrive.CurPIDMove({4.3*cell, 0.7*cell,0},50, head);
}

void PushBallsTwice_base(){
    Hdrive.moveInches(0.5*cell, 30, 1000);
    Hdrive.CurPIDMove({3.8*cell, 2.3*cell,0},40,false,3000);
    Hdrive.turnToAngle(0, 30, 1000, false);
    Hdrive.moveInches(0.3*cell, 40, 2000);
    Hdrive.simpleMove(-80, 0.5);
    // move back
    odom.setXY(5.7*cell, 1.6*cell);
    // odom.setPosition(5.7*cell, 1.6*cell, -180);

    Hdrive.turnToAngle(-90, 30, 2000, false);
    // Hdrive.turnToAngle(-80, 30, 2000, false);

    Hdrive.CurPIDMove({4.9*cell, 0.8*cell,0},30);
    Hdrive.turnToAngle(-87, 30, 2000, false);
    Hdrive.moveInches(0.4*cell, 40, 2000, false);
    // again
    Hdrive.CurPIDMove({6.5*cell, 4*cell,0},30,false,3000);
    Hdrive.turnToAngle(0, 30, 1000, false);
    Hdrive.moveInches(0.3*cell, 40, 2000);
    task::sleep(600);
    Hdrive.simpleMove(-80, 0.5);
}



int TestSkill(){
    CrossMid2();
    // PushBallsTwice();
    Hdrive.moveInches(0.5*cell, 50, 1000);
    Hdrive.CurPIDMove({3.8*cell, 2.7*cell,0},50,false,3000);

    Hdrive.turnToAngle(0, 50, 1000, false);
    Hdrive.moveInches(0.3*cell, 40, 2000);
    // Hdrive.turnToAngle(-10, 30, 2000, false);
    Hdrive.simpleMove(-80, 0.8);
    // move back
    odom.setXY(5.7*cell, 1.6*cell);
    // odom.setPosition(5.7*cell, 1.6*cell, -180);
    Hdrive.turnToAngle(-90, 40, 1000, false);
    Hdrive.CurPIDMove({4.9*cell, 0.8*cell,0},30);
    intake();
    Hdrive.turnToAngle(-87, 30, 2000, false);
    Hdrive.moveInches(0.4*cell, 40, 2000, false);
    // again
    Hdrive.CurPIDMove({6.5*cell, 4*cell,0},30,false,3000);
    // spit();
    Hdrive.turnToAngle(0, 40, 2000, false);
    Hdrive.moveInches(0.3*cell, 40, 2000);
    task::sleep(600);
    Hdrive.simpleMove(-80, 0.8);
    
    // Go to mid
    odom.setXY(5.7*cell, 1.6*cell);
    // odom.setPosition(5.7*cell, 1.6*cell, 180);

    Hdrive.CurPIDMove({5.7*cell, 1.2*cell, 0}, 25, true, 1500);
    Hdrive.moveToTarget({4.6*cell, 1.6*cell, 0}, 36, 30, 4000, 1);
    sideopen();
    intake();

    // Hdrive.turnToTarget({3.5*cell, 3*cell, 0}, 36, 2000, 1);
    Hdrive.CurPIDMove({5*cell, 2.5*cell, 0}, 25, 1, 2000);
    
    //Hdrive.turnToAngle(-120, 50, 2000, 1);
    //task::sleep(100);
    Hdrive.turnToAngle(-90, 30, 2000, 1);
    spit();
    Hdrive.simpleMove(60, 1);

    for(int i=5;i>0;i--){
        sideclose();
        Hdrive.moveInches(0.3*cell, 30, 1000, false);
        task::sleep(200);  
        sideopen();
        Hdrive.moveInches(0.5*cell, 60, 1000, true);
    }

    return 1;
}

int TestMove4(){
    kickColBall();

    Hdrive.CurPIDMove({1.1*cell,0.8*cell,0}, 30);
    Hdrive.turnToAngle(0, 30, 2000, 1);

    loading.continuousKick(7);

    TestMove5();
    
    return 0;
}

void Version3copy(){    
    Hdrive.setStop(vex::brakeType::hold);
    loading.setPosition(LEFT);
    loading.kickRight();
    loading.continuousKick(10);

    // CrossMid
    // the position after loading is {23,25.7,0}
    odom.setPosition(0.95*cell, 0.8*cell,0);
    // Hdrive.CurPIDMove({0.5*cell, 1.1*cell,0},20);  
    // Hdrive.CurTimeMove(0.05, 20, 1000);
    Hdrive.turnToAngle(45, 30, 1000, 1);

    // Hdrive.CurPIDMove({0.6*cell, 1.4*cell,0},30);
    // {22,35}
    Hdrive.CurPIDMove({1.3*cell, 0.5*cell,0},30,false);
    Hdrive.turnToAngle(84,30,2000,1);
    Hdrive.CurPIDMove({3*cell, 0.5*cell,0},45,false);
    // Hdrive.moveInches(1.9*cell, 50, 3000, false);

    // PushBallsTwice
    Hdrive.moveInches(0.5*cell, 30, 1000);
    Hdrive.CurPIDMove({3.8*cell, 2.3*cell,0},40,false,3000);
    Hdrive.turnToAngle(0, 30, 1000, false);
    Hdrive.moveInches(0.3*cell, 40, 2000);
    Hdrive.simpleMove(-80, 0.5);
    // move back
    odom.setXY(5.7*cell, 1.6*cell);
    // odom.setPosition(5.7*cell, 1.6*cell, -180);
    Hdrive.turnToAngle(-90, 30, 2000, false);
    Hdrive.CurPIDMove({4.9*cell, 0.8*cell,0},30);
    Hdrive.turnToAngle(-87, 30, 2000, false);
    Hdrive.moveInches(0.4*cell, 40, 2000, false);
    // again
    Hdrive.CurPIDMove({6.5*cell, 4*cell,0},30,false,3000);
    Hdrive.turnToAngle(0, 30, 1000, false);
    Hdrive.moveInches(0.3*cell, 40, 2000);
    task::sleep(600);
    Hdrive.simpleMove(-80, 0.5);

    // MoveBackForAWP{
    odom.setXY(5.7*cell, 1.6*cell);
    Hdrive.CurPIDMove({4.3*cell, 0.8*cell,0},40);
}

// load and push
int Version1(){
    loading.setPosition(LEFT);
    loading.kickRight();
    loading.continuousKick(10);
    TestMove3();
    return 1;
}

int Version2(){
    TestMove();
    TestBack();
    // stop();
    TestMove3();
    return 1;
}

// twice push
int Version3(){
    Hdrive.setStop(vex::brakeType::hold);
    loading.setPosition(LEFT);
    loading.kickRight();
    loading.continuousKick(10);
    CrossMid();
    PushBallsTwice();
    MoveBackForAWP();
    return 1;
}


// int main()
// {    
//     pre_auton();
//     // Hdrive.setStop(vex::brakeType::hold);
//     // loading.setPosition(LEFT);
//     // loading.kickRight();
//     // loading.continuousKick(10, 600);

//     // TestGetABall();
//     // CrossMid3();
//     // PushBallsTwice();
//     // Hdrive.moveInches(0.4*cell, 40, 1500);
//     // Hdrive.turnToAngle(0, 40, 2000, 1);
//     // spit();
//     // task::sleep(200);
//     // Hdrive.simpleMove(80, 0.7);

//     // MoveBackForAWP(false);

//     autonomous();
//     // skillautonoumous();
//     usercontrol();
//     return 0;
// }

int main()
{
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
    while (true)
    {
        wait(100, msec);
    }
}

void MovementAfterLoading(){
    CrossMid3();
    PushBallsTwice();

    // the ball in front
    Hdrive.moveInches(0.4*cell, 40, 1000);
    Hdrive.turnToAngle(-4, 40, 1000, 1);
    spit();
    // task::sleep(200);
    Hdrive.simpleMove(80, 0.7);

    MoveBackForAWP(false);
}

void autonomous()
{
    task::sleep(3000);//手柄进自动线程需要sleep3000秒
    kickColBall();

    Hdrive.CurPIDMove({1.1*cell,0.8*cell,0}, 30);
    Hdrive.turnToAngle(0, 30, 2000, 1);

    loading.continuousKick(10);

    Hdrive.turnToAngle(70, 40, 1500, true);
    Hdrive.simpleMove(-70, 2);
    Hdrive.turnToAngle(90, 40, 1500, true);
    Hdrive.turnToAngle(-50, 40, 1500, false);
    Hdrive.simpleMove(-40, 1.5);
    //撞击第二次

    Hdrive.simpleMove(40,1.5);
    Hdrive.simpleMove(-40, 1.5);

    Hdrive.turnToAngle(0, 40, 1500, false);
    Hdrive.moveInches(0.3*cell, 40, 1200);
    Hdrive.simpleMove(-50, 1);

    MoveBackForAWP();

    Hdrive.moveInches(1*cell, 40, 2000);

}
void skillautonoumous()
{
    
}


void usercontrol()
{
    Controller1.ButtonUp.pressed([]() {        
        side_bar.spin(reverse); // 电机正转
    });

    Controller1.ButtonUp.released([]() {        
        side_bar.stop(hold);
    });
    Controller1.ButtonDown.pressed([]() {        
         side_bar.spin(vex::forward); // 电机正转
    });

    Controller1.ButtonDown.released([]() {        
        side_bar.stop(hold);
    });


    Controller1.ButtonLeft.pressed([]() {    
        loading.kickLeft();  
        // side_bar.setVelocity(100, pct);  
        // side_bar.spinFor(400, msec);
        // //vexMotorVoltageSet(side_bar.index(), 100 * 12000.0 / 100.0);
        // task::sleep(600);
        // side_bar.stop(hold);
    });

    Controller1.ButtonRight.pressed([]() {        
        loading.kickRight();
        // side_bar.spin(vex::reverse); // 电机正转
        // //vexMotorVoltageSet(side_bar.index(), -100 * 12000.0 / 100.0);
        // //side_bar.setVelocity(100, pct);

        // task::sleep(450);
        // side_bar.stop(hold);
    });


    Controller1.ButtonR1.pressed([]() {
        static bool motorRunning = false; // 用于追踪电机状�?
  
        if (!motorRunning) {
            cheel.spin(forward,70,pct);
        } else {
           cheel.stop();// 停�?�电机旋�??
        }
        motorRunning = !motorRunning; // 切换电机状态}
    });
    Controller1.ButtonR2.pressed([]() {
        static bool motorRunning = false; // 用于追踪电机状�?
  
        if (!motorRunning) {
            cheel.spin(vex::reverse,70,pct);
        } else {
            cheel.stop();// 停�?�电机旋�??
        }
        motorRunning = !motorRunning; // 切换电机状态}
    });

    Controller1.ButtonL1.pressed([]() {
        static bool side_state = false;
  
        if (!side_state) {
            sideopen();
        } else {
            sideclose();
        }
        side_state = !side_state;
    });    
    Controller1.ButtonL2.pressed([]() {
        static bool gas_state = false; // 用于追踪电机状�?
  
        if (!gas_state) {
            gasopen();
        } else {
           gasclose();// 停�?�电机旋�??
        }
        gas_state = !gas_state; // 切换电机状态}
    });

    Controller1.ButtonA.pressed([]() {
        static bool motorRunning = false; // 用于追踪电机状�?
  
        if (!motorRunning) {
            gas.state(100, pct);
        } else {
            gas.state(0, pct);
        }
        motorRunning = !motorRunning; // 切换电机状态}
    });

    Controller1.ButtonY.pressed([]() {
        static bool ready = false;
        if(!ready) loading.kickLeft();
        else loading.kickRight();
        loading.hold();
    });

    Controller1.ButtonB.pressed([]() {
        Hdrive.setStop(vex::brakeType::hold);
    });
    
    while (true)
    {
        Hdrive.ArcadeDrive();
        // double currentAngle = imu.rotation();
        // currentAngle = Math::getWrap360(currentAngle);
        // printf("imu : %lf\n", currentAngle);

        // 调试时通过按键进入�??�??
        // if (Controller1.ButtonY.pressing())
        // {
        //     autonomous();
        // }
        // if(Controller1.ButtonY.pressing()){
        //     skillautonoumous();
        // }
    }
}
