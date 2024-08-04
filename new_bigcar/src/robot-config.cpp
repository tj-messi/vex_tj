#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// 底盘电机
motor L1 = motor(PORT2, ratio6_1, true);
motor L2 = motor(PORT4, ratio6_1, false);
motor L3 = motor(PORT7, ratio6_1, true);
motor L4 = motor(PORT8, ratio6_1, false);
motor R1 = motor(PORT10, ratio6_1, false);
motor R2 = motor(PORT12, ratio6_1, true);
motor R3 = motor(PORT14, ratio6_1, false);
motor R4 = motor(PORT16, ratio6_1, true); 

motor lift_armMotorA = motor(PORT11, ratio36_1, true);
motor lift_armMotorB = motor(PORT13, ratio36_1, false);
motor_group lift_arm = motor_group(lift_armMotorA, lift_armMotorB);

motor rollerMotorA = motor(PORT15, ratio18_1, true);
motor rollerMotorB = motor(PORT17, ratio18_1, false);
motor_group roller_group = motor_group(rollerMotorA, rollerMotorB);

controller Controller1 = controller(primary);

// encoder encoderHorizonal = encoder(Brain.ThreeWirePort.G);
// encoder encoderVertical = encoder(Brain.ThreeWirePort.A);

motor side_bar = motor(PORT20, ratio18_1, false);

inertial imu = inertial(PORT19, left);  // 第二个参数要写left

pwm_out gas1 = pwm_out(Brain.ThreeWirePort.A);

pwm_out gas2 = pwm_out(Brain.ThreeWirePort.B);
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