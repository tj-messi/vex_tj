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
extern const double r_wheel_encoder;
// GPS的x轴方向偏置 
extern const double gps_offset_x;      
// GPS的y轴方向偏置                
extern const double gps_offset_y;
// 编码轮旋转角度
extern const double encoder_rotate_degree;
extern double gps_x;
extern double gps_y;
extern double gps_heading;                    
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

extern motor_group lift_arm;
extern motor_group roller_group;

extern controller Controller1;
extern motor side_bar;

extern vex::message_link AllianceLink;

extern encoder encoderHorizonal;
extern encoder encoderVertical;

extern pwm_out gas_push;
extern pwm_out gas_lift;
extern pwm_out gas_hold;

extern inertial imu;

extern distance DistanceSensor;

extern vex::gps GPS_;

extern vision Vision;

extern motor convey_beltMotor;

// 四角底盘电机组声明
extern std::vector<vex::motor*> _leftMotors;
extern std::vector<vex::motor*> _rightMotors;

// 八角底盘电机组声明
extern std::vector<vex::motor*> _lfMotors;
extern std::vector<vex::motor*> _lbMotors;
extern std::vector<vex::motor*> _rfMotors;
extern std::vector<vex::motor*> _rbMotors;
// extern pwm_out gas;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );