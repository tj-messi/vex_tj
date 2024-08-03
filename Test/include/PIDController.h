#include<math.h>
class PIDController {
private:
  double Kp; // 比例增益
  double Ki; // 积分增益
  double Kd; // 微分增益
  double integral; // 积分项
  double last_error; // 上一次的误差

public:
    // PID控制器的构造函数
    PIDController(double Kp, double Ki, double Kd)
        : Kp(Kp), Ki(Ki), Kd(Kd), integral(0.0), last_error(0.0) {}

    // 计算PID输出
    double computePID(double setpoint, double input, double dt) {
        double error = setpoint - input; // 计算误差
        integral += error * dt; // 计算积分项
        double derivative = (error - last_error) / dt; // 计算微分项
        double output = Kp * error + Ki * integral + Kd * derivative; // 计算PID输出
        last_error = error; // 更新最后的错误
        return output; // 返回PID输出
    }

    // 重置积分项
    void resetIntegral() {
        integral = 0.0;
        last_error = 0.0;
    }


};