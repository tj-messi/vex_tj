#include "vex.h"

using namespace vex;

/*************************************

        physical configurations

*************************************/
const double car_width = 11.15;                   // 轮距
const double r_wheel = 4 / 2;                     // 车轮半径
const double gear_ratio = 0.667;                  // 底盘电机-轮的齿轮传动比（加速配比就大于1，减速配比就小于1）
const double r_motor = r_wheel * gear_ratio ;     // 电机转角-电机转周的换算比
const double cell  = 24;                          // 一个地垫长度(inches)
const double hOffset  = -5;                    // 里程计偏置（inches）----从旋转中心向里程计轮延伸方向作垂线
const double vOffset  = 5;                    // 里程计偏置（inches）----从旋转中心向里程计轮延伸方向作垂线
const double r_wheel_encoder = 2.75 / 2;          // 编码轮周长
const double gps_offset_x = 0;                    // GPS的x轴方向偏置 
const double gps_offset_y = 6.7;                    // GPS的y轴方向偏置 
const double encoder_rotate_degree = 45;          // 编码轮旋转角度
double gps_x = 0;
double gps_y = 0;
double gps_heading = 0;
/*************************************

            VEX devices

*************************************/
// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
// 底盘电机 - 四角底盘
motor L1 = motor(PORT13, ratio6_1, false);
motor L2 = motor(PORT13, ratio6_1, true);
motor L3 = motor(PORT13, ratio6_1, false);
motor L4 = motor(PORT13, ratio6_1, true);
motor R1 = motor(PORT13, ratio6_1, true);
motor R2 = motor(PORT13, ratio6_1, false);
motor R3 = motor(PORT13, ratio6_1, true);
motor R4 = motor(PORT13, ratio6_1, false); 
std::vector<vex::motor*> _leftMotors = {&L1, &L2, &L3,  &L4};
std::vector<vex::motor*> _rightMotors = {&R1, &R2, &R3, &R4};

// 底盘电机 - 八角底盘
motor lf1 = motor(PORT1, ratio18_1, false);
motor lf2 = motor(PORT2, ratio18_1, true);
motor lb1 = motor(PORT3, ratio18_1, false);
motor lb2 = motor(PORT4, ratio18_1, true);
motor rf1 = motor(PORT10, ratio18_1, false);
motor rf2 = motor(PORT9, ratio18_1, true);
motor rb1 = motor(PORT11, ratio18_1, false);
motor rb2 = motor(PORT12, ratio18_1, true);

std::vector<vex::motor*> _lfMotors = {&lf1, &lf2};
std::vector<vex::motor*> _lbMotors = {&lb1, &lb2};
std::vector<vex::motor*> _rfMotors = {&rf1, &rf2};
std::vector<vex::motor*> _rbMotors = {&rb1, &rb2};

// 抬升臂
motor lift_armMotorA = motor(PORT8, ratio36_1, true);
motor lift_armMotorB = motor(PORT15, ratio36_1, false);
motor_group lift_arm = motor_group(lift_armMotorA, lift_armMotorB);

// 传送带
motor convey_beltMotor = motor(PORT18, ratio36_1, false);

// 吸球
motor rollerMotorA = motor(PORT6, ratio18_1, true);
motor rollerMotorB = motor(PORT13, ratio18_1, false);
motor_group roller_group = motor_group(rollerMotorA, rollerMotorB);
// 遥控器
controller Controller1 = controller(primary);
// 通信天线
vex::message_link AllianceLink(PORT13, "tju1", linkType::worker);
// 里程计
encoder encoderHorizonal = encoder(Brain.ThreeWirePort.A);
encoder encoderVertical = encoder(Brain.ThreeWirePort.G);
// 导入杆
motor side_bar = motor(PORT13, ratio18_1, false);
// imu惯性传感器
inertial imu = inertial(PORT17);  // 第二个参数要写right

// 气动件
pwm_out gas_push = pwm_out(Brain.ThreeWirePort.D);
pwm_out gas_lift = pwm_out(Brain.ThreeWirePort.E);
pwm_out gas_hold = pwm_out(Brain.ThreeWirePort.F);

// 距离传感器
distance DistanceSensor = distance(PORT13);
// gps
gps GPS_ = gps(PORT16, 0, 0, inches, 0);

// vision signature
vision::signature Red = vision::signature(1, 32717, 32819, 32768, -51, 51, 0, 16711680, 0);
vision::signature Blue = vision::signature(2, -14635, -14349, -14492, 3405, 3831, 3618, 34985, 0);
// vision
vision Vision = vision(PORT19, 50, Red, Blue);


// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/*************************************

            axis defination

*************************************/
/*

robot_local : 

                  ^ y   Head  ^    ^--->
                  |           |    |<-- imu  
                  |
                  |
(rotation_center) |——————————> x


robot_global : 

                  ^ y
                  |           
                  |
                  |
         (middle) |——————————> x

*/

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}