using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor L1;
extern motor L2;
extern motor L3;
extern motor R1;
extern motor R2;
extern motor R3;
extern controller Controller1;
extern motor cheel;
extern motor side_bar;

extern inertial imu;

extern pwm_out gas;
extern pwm_out side_gas;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );