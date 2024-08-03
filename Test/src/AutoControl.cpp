#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"
#include "PIDController.h"

using namespace vex;

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//  璁剧疆鍓嶈繘鐨勬帶鍒跺弬鏁?
const double forward_parameter = 1.0;
// 璁剧疆杞姩鐨凱ID鎺у埗鍙傛暟
const double Kp = 0.15;   // 璇樊椤癸紝涔熻鍋忓ぇ閫備腑涓€鐐癸紝璺熷洖澶嶅拰鍓嶈繘閮芥湁鍏?
const double Ki = 0.001;  // 鍔犻€熷害锛屽敖閲忚皟鐨勫皬涓€浜涳紝鍚﹀垯浼氬ぇ鎽嗛敜
const double Kd = 0.15;  // 褰卞搷鍥炲鐨勯€熷害锛岀敱浜庡ぇ閮ㄥ垎鎯呭喌鍥炲鐨勬瘮杈冩參锛屽洜姝ゅ彲浠ョ◢寰皟澶т竴鐐?
// 璁剧疆杞悜鐨勯€熷害閲?
const double initial_speed = 1.0;
const double max_speed = 2.0; 
const double adversial_speed = 3.0;

class AutoControl{
public:

  // 搴曠洏鐨勭墿鐞嗘帶鍒跺眰
  static void RunPhysicalControl(double l, double r) { 
    Left1.spin(fwd, l, pct);
    Left2.spin(fwd, l, pct);
    Right1.spin(fwd, r, pct);
    Right2.spin(fwd, r, pct);
  }

  // 鍚戝墠杩涚殑閫昏緫鎺у埗
  static void forward(directionType direct, double dist, double speed){  // direct: fwd rev
    const double K = forward_parameter;//瀵瑰簲甯告暟锛屽彲浠ヨ繘琛岃皟璇曞緱鍒版渶缁堢殑闀垮害
    double run_time = dist/speed*K ;
    speed *= (direct==fwd?1:-1); // 妫€娴嬫柟鍚?fwd鏄墠杩涳紝rev 鏄悗閫€
    RunPhysicalControl(speed,speed);
    task::sleep(run_time);  // 杩愯鐨勬椂闀?
    RunPhysicalControl(0,0);
  }

  // 杞悜鐨勯€昏緫鎺у埗
  static void rotate(double dest_degree, double speed){  
    
    // 鍒濆鍖朠ID鎺у埗鍣?
    PIDController pid(Kp, Ki, Kd);
    // 鑾峰緱褰撳墠瑙掑害
    double current_degree = imu.angle(deg);
    // 鑾峰緱褰撳墠瑙掑害涓庣洰鏍囪搴︾殑宸€硷紝杩欓噷鍖呭惈浜嗗浼樿鍜屽姡瑙掔殑澶勭悊閫昏緫
    double error = 
    fabs(dest_degree - current_degree) <= 180 ? 
     (dest_degree - current_degree) 
    :( 
      (dest_degree - current_degree)>0 ? (dest_degree - current_degree) - 360 : (dest_degree - current_degree) + 360
    );
    // 鍋囪绱Н璇樊鐨勬洿鏂板懆鏈熸槸0.01
    double dt = 0.01;
    int count = 0;
    // 鎺у埗寰幆锛屽ぇ浜?搴︾殑鏃跺€欏氨涓€鐩存帶鍒?
    while(1){
      if(fabs(error)<0.5){
        count++;
      }
      if(count == 5){
        break;
      }
      Brain.Screen.clearLine();
      Brain.Screen.print("angle");
      Brain.Screen.print(current_degree);
      Brain.Screen.print(" speed:");
      Brain.Screen.print(speed);
      Brain.Screen.print(" error:");
      Brain.Screen.print(error);
      // 鑾峰緱褰撳墠瑙掑害
      current_degree = imu.angle(deg);
      // 鏍规嵁PID鎺у埗鍣ㄨ幏寰楁柊閫熷害
      speed = pid.computePID(error, dt);
      speed = fabs(speed) < max_speed ? speed : (double)( speed / fabs(speed) ) * max_speed;  // 闃堝€奸檺閫燂紝娉ㄦ剰淇濈暀鏂瑰悜
      // 鏇存柊閫熷害
      RunPhysicalControl(speed,-1*speed);
      // 鏇存柊褰撳墠璇樊
      current_degree = imu.angle(deg);
      error = fabs(dest_degree - current_degree) >= 180 ? 
      (dest_degree - current_degree) 
      :( 
      (dest_degree - current_degree)>0 ? (dest_degree - current_degree) - 360 : (dest_degree - current_degree) + 360
       );
    }

    pid.resetIntegral();
    RunPhysicalControl(0.0, 0.0); // 鍋滄鐢垫満

  }
  

  // 杩欓噷搴曠洏鍜屽す瀛愮殑鑷姩鎺у埗绋嬪簭鏄悇鍐欏悇鐨?
  static int AutoBaseMove(){  // 搴曠洏鐨勮嚜鍔ㄦ帶鍒剁▼搴?
    //forward(fwd, 10000, 20);
    task::sleep(100);

    rotate(90, initial_speed);
    task::sleep(1000);
    rotate(180, initial_speed);
    task::sleep(1000);
    rotate(60, initial_speed);
    task::sleep(1000);
    rotate(270, initial_speed);
    task::sleep(1000);
    rotate(45, initial_speed);
    task::sleep(1000);
    rotate(2, initial_speed);

    return 0;
  } 

  // 澶ц噦鍙婂す瀛愮殑鐗╃悊鎺у埗
  static void clawPhysicalMove(double a1, double a2, double c) { // 澶瑰瓙鐨勭墿鐞嗘帶鍒跺眰
    arm1.spin(fwd,a1,pct);
    arm2.spin(fwd,a2,pct);
    clip.spin(fwd,c,pct);
  }
  static int clawmove(double dest_degree, double speed){ //澶ц噦鑷姩鎺у埗

    // 鍒濆鍖栧皢鐢垫満璁板綍鐨勮搴﹀綊闆?
    arm1.resetPosition();
    arm2.resetPosition();

    // 鍒濆鍖朠ID鎺у埗鍣?
    PIDController pid(Kp, Ki, Kd);
    // 鑾峰緱褰撳墠瑙掑害
    double current_degree = imu.angle(deg);
    // 鑾峰緱褰撳墠瑙掑害涓庣洰鏍囪搴︾殑宸€硷紝杩欓噷鍖呭惈浜嗗浼樿鍜屽姡瑙掔殑澶勭悊閫昏緫
    double error = 
    fabs(dest_degree - current_degree) <= 180 ? 
     (dest_degree - current_degree) 
    :( 
      (dest_degree - current_degree)>0 ? (dest_degree - current_degree) - 360 : (dest_degree - current_degree) + 360
    );
    // 鍋囪绱Н璇樊鐨勬洿鏂板懆鏈熸槸0.01
    double dt = 0.01;
    int count = 0;
    // 鎺у埗寰幆锛屽ぇ浜?搴︾殑鏃跺€欏氨涓€鐩存帶鍒?

    while(1){
      if(fabs(error)<0.5){
        count++;
      }
      if(count == 5){
        break;
      }
    
      // 鑾峰緱褰撳墠瑙掑害
      current_degree = arm1.position(deg);
      // 鏍规嵁PID鎺у埗鍣ㄨ幏寰楁柊閫熷害
      speed = pid.computePID(error, dt);
      speed = fabs(speed) < max_speed ? speed : (double)( speed / fabs(speed) ) * max_speed;  // 闃堝€奸檺閫燂紝娉ㄦ剰淇濈暀鏂瑰悜
      // 鏇存柊閫熷害
      clawPhysicalMove(speed, speed, 0);
      // 鏇存柊褰撳墠璇樊
      current_degree = arm1.position(deg);
      error = fabs(dest_degree - current_degree) >= 180 ? 
      (dest_degree - current_degree) 
      :( 
      (dest_degree - current_degree)>0 ? (dest_degree - current_degree) - 360 : (dest_degree - current_degree) + 360
       );
    }

    pid.resetIntegral();
    RunPhysicalControl(0.0, 0.0); // 鍋滄鐢垫満

    return 0;

  }
  
  void clawPhysicalStop(){

  }

  static int AutoClawMove(){   // 澶瑰瓙鐨勮嚜鍔ㄦ帶鍒剁▼搴?

    return 0;
  } 
  static int AutoArmMove(){   // 澶瑰瓙鐨勮嚜鍔ㄦ帶鍒剁▼搴?

    return 0;
  } 

  static int AutoMove(){

    vex::task autobasemove(AutoBaseMove); 
    vex::task autoclawmove(AutoClawMove); 
    vex::task autoarmmove(AutoArmMove); 

    return 0;
  }
};