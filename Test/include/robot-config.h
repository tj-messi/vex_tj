using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor_group Left1;
extern motor_group Left2;
extern motor_group Right1;
extern motor_group Right2;
extern inertial imu;
extern motor_group arm1;
extern motor_group clip;
extern motor_group arm2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );