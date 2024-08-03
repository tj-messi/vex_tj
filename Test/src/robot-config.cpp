#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor Left1MotorA = motor(PORT3, ratio18_1, true);
motor Left1MotorB = motor(PORT2, ratio18_1, false);
motor_group Left1 = motor_group(Left1MotorA, Left1MotorB);
motor Left2MotorA = motor(PORT4, ratio18_1, true);
motor Left2MotorB = motor(PORT5, ratio18_1, false);
motor_group Left2 = motor_group(Left2MotorA, Left2MotorB);
motor Right1MotorA = motor(PORT7, ratio18_1, false);
motor Right1MotorB = motor(PORT6, ratio18_1, true);
motor_group Right1 = motor_group(Right1MotorA, Right1MotorB);
motor Right2MotorA = motor(PORT8, ratio18_1, false);
motor Right2MotorB = motor(PORT9, ratio18_1, true);
motor_group Right2 = motor_group(Right2MotorA, Right2MotorB);
inertial imu = inertial(PORT11);
motor arm1MotorA = motor(PORT18, ratio36_1, true);
motor arm1MotorB = motor(PORT1, ratio36_1, false);
motor_group arm1 = motor_group(arm1MotorA, arm1MotorB);
motor clipMotorA = motor(PORT14, ratio18_1, false);
motor clipMotorB = motor(PORT15, ratio18_1, false);
motor_group clip = motor_group(clipMotorA, clipMotorB);
motor arm2MotorA = motor(PORT10, ratio36_1, true);
motor arm2MotorB = motor(PORT19, ratio36_1, false);
motor_group arm2 = motor_group(arm2MotorA, arm2MotorB);

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