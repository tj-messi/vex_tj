#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// 底盘电机
motor L1 = motor(PORT2, ratio6_1, true);
motor L2 = motor(PORT5, ratio6_1, true);
motor L3 = motor(PORT6, ratio6_1, true);
motor R1 = motor(PORT13, ratio6_1, false);
motor R2 = motor(PORT17, ratio6_1, false);
motor R3 = motor(PORT19, ratio6_1, false);

controller Controller1 = controller(primary);

motor cheel = motor(PORT15, ratio6_1, true);
motor side_bar = motor(PORT12, ratio36_1, false);

inertial imu = inertial(PORT16, left);  // 第二个参数要写left

// 气动件
pwm_out side_gas = pwm_out(Brain.ThreeWirePort.G);
pwm_out gas = pwm_out(Brain.ThreeWirePort.H);

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