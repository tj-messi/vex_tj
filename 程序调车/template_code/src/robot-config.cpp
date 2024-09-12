#include "vex.h"

using namespace vex;

/*************************************

        physical configurations

*************************************/
const double car_width = 11.15;                 // 轮距
const double r_wheel = 3.25 / 2;                // 车轮半径
const double gear_ratio = 0.667;                // 底盘电机-轮的齿轮传动比（加速配比就大于1，减速配比就小于1）
const double r_motor = r_wheel * gear_ratio ;   // 电机转角-电机转周的换算比
const double cell  = 24;                        // 一个地垫长度(inches)
const double hOffset  = 0.5;                    // 里程计偏置（inches）----从旋转中心向里程计轮延伸方向作垂线
const double vOffset  = 0.5;                    // 里程计偏置（inches）----从旋转中心向里程计轮延伸方向作垂线
const double encoderCircumference = 2.57;       // 编码轮周长

/*************************************

            VEX devices

*************************************/
// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
// 底盘电机
motor L1 = motor(PORT1, ratio6_1, false);
motor L2 = motor(PORT2, ratio6_1, true);
motor L3 = motor(PORT4, ratio6_1, false);
motor L4 = motor(PORT7, ratio6_1, true);
motor R1 = motor(PORT11, ratio6_1, true);
motor R2 = motor(PORT12, ratio6_1, false);
motor R3 = motor(PORT19, ratio6_1, true);
motor R4 = motor(PORT20, ratio6_1, false); 
std::vector<vex::motor*> _leftMotors = {&L1, &L2, &L3,  &L4};
std::vector<vex::motor*> _rightMotors = {&R1, &R2, &R3, &R4};
// 抬升臂
motor lift_armMotorA = motor(PORT10, ratio36_1, true);
motor lift_armMotorB = motor(PORT13, ratio36_1, false);
motor_group lift_arm = motor_group(lift_armMotorA, lift_armMotorB);
// 吸球
motor rollerMotorA = motor(PORT15, ratio18_1, true);
motor rollerMotorB = motor(PORT17, ratio18_1, false);
motor_group roller_group = motor_group(rollerMotorA, rollerMotorB);
// 遥控器
controller Controller1 = controller(primary);
// 通信天线
vex::message_link AllianceLink(PORT3, "tju1", linkType::worker);
// 里程计
encoder encoderHorizonal = encoder(Brain.ThreeWirePort.G);
encoder encoderVertical = encoder(Brain.ThreeWirePort.A);
// 导入杆
motor side_bar = motor(PORT14, ratio18_1, false);
// imu惯性传感器
inertial imu = inertial(PORT16, left);  // 第二个参数要写left
// 气动件
pwm_out gas1 = pwm_out(Brain.ThreeWirePort.A);
pwm_out gas2 = pwm_out(Brain.ThreeWirePort.B);
// 碰撞传感器
bumper  collision_sensor =  bumper(Brain.ThreeWirePort.F);
// 距离传感器
distance DistanceSensor = distance(PORT6);
// gps
gps GPS = gps(PORT10, 10.00, 10.00, mm, 180);
// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}