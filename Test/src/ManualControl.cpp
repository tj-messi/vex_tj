#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"

using namespace vex;

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
class ManualControl{
public:

  static void Stop(){  // 鐢垫満閿佹
    while(1){
        Left1.setStopping(brakeType::hold);    
        Left2.setStopping(brakeType::hold);   
        Right1.setStopping(brakeType::hold);   
        Right2.setStopping(brakeType::hold);
        arm1.setStopping(brakeType::hold);
        arm2.setStopping(brakeType::hold);
        clip.setStopping(brakeType::hold); 
    }
  }

  static void basemovePhysicalControl(double l, double r) { // 搴曠洏鐨勭墿鐞嗘帶鍒跺眰
    Left1.spin(fwd, l, pct);
    Left2.spin(fwd, l, pct);
    Right1.spin(fwd, r, pct);
    Right2.spin(fwd, r, pct);
  }

  static int basemoveLogicalControl(){  // 搴曠洏鐨勯€昏緫鎺у埗灞?
    while(1){

      int f, s;
      f = Controller1.Axis3.value();  // 鑾峰緱鍓嶈繘鐨勫弬鏁?
      s = Controller1.Axis1.value();  // 鑾峰緱杞悜鐨勫弬鏁?
      f = abs(f)>15?f:0;
      s = abs(s)>15?s:0;
      if(f!=0||s!=0) basemovePhysicalControl((f+s)*70.0/127.0,(f-s)*70.0/127.0);
      else basemovePhysicalControl(0, 0);
      
      task::sleep(1);
    }
    return 0;
  }

  static void acmovePhysicalControl(double a1, double a2) { // 澶瑰瓙鐨勭墿鐞嗘帶鍒跺眰

    arm1.spin(fwd,a1,pct);
    arm2.spin(fwd,a2,pct);
  
  }
  static void clipmovePhysicalControl(double c){
      clip.spin(fwd,c,pct);
  }

  static int clawmoveLogicalControl(){ // 澶瑰瓙鐨勯€昏緫鎺у埗灞?

    //a鏄ぇ鑷傚弬鏁帮紝c鏄す瀛愬弬鏁?
    int a1 = 0;  
    int a2 = 0;  
    int c = 0;  

    while(1) {

      // 澶瑰瓙鎺у埗
      if(Controller1.ButtonR1.pressing()){  c=8; }        // 澶瑰瓙姝ｈ浆
      else if(Controller1.ButtonR2.pressing()){  c=-8; }  // 澶瑰瓙鍙嶈浆
      else{ c = 0; }       

      // 澶ц噦鎺у埗
      if(Controller1.ButtonL1.pressing()){  a1=18;  a2=18;  }  // 澶ц噦姝ｈ浆
      if(Controller1.ButtonL2.pressing()){  a1=-18; a2 = -18;  }  // 澶ц噦姝ｈ浆
      else{  a1 = 0; a2 = 0; }     // 涓嶆寜閿洿鎺ユ椂鍋?

      if(a1&&a2){
        acmovePhysicalControl(a1, a2);
      }else{
        arm1.setStopping(brakeType::hold);
        arm2.setStopping(brakeType::hold);
      }

      if(c){
        clipmovePhysicalControl(c);
      }
        
      task::sleep(1);

    }  
     return 0;
  }
  
  static int Manualcontrol(){

    vex::task manualbasemove(basemoveLogicalControl); 
    vex::task manualclawmove(clawmoveLogicalControl); 

    return 0;
  }
};
