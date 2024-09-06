using namespace vex;

/*************************************

        physical configurations

*************************************/
// 轮距
extern const double car_width;
// 车轮半径
extern const double r_wheel;
// 底盘电机-轮的齿轮传动比
extern const double gear_ratio;
// 电机转角-电机转周的换算比
extern const double r_motor; 
// 一个地垫长度(inches)
extern const double cell;
// 里程计偏置（inches）----做垂线
extern const double hOffset;
extern const double vOffset;
// 编码轮周长
extern const double encoderCircumference;

/*************************************

            VEX devices

*************************************/
extern brain Brain;
extern motor L1;
extern motor L2;
extern motor L3;
extern motor L4;
extern motor R1;
extern motor R2;
extern motor R3;
extern motor R4;
extern std::vector<vex::motor*> _leftMotors;
extern std::vector<vex::motor*> _rightMotors;

extern motor_group lift_arm;
extern motor_group roller_group;

extern controller Controller1;
extern motor side_bar;

extern vex::message_link AllianceLink;

extern encoder encoderHorizonal;
extern encoder encoderVertical;

extern pwm_out gas1;
extern pwm_out gas2;

extern inertial imu;

extern bumper  collision_sensor;

extern distance DistanceSensor;

extern gps GPS;
// extern pwm_out gas;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );