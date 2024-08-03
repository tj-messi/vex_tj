#include "v5.h"
#include "v5_vcs.h"
#include "robot-config.h"
#include "ManualControl.cpp"
#include "AutoControl.cpp"
using namespace vex;

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int mode = 1; // mode 鍒嗕负涓ょ妯″紡锛氳嚜鍔ㄦā寮忥紙2锛夊拰鎵嬪姩妯″紡锛?锛?

class Comp{
public:

  static int ModeChoose(){  // 璋冩暣鎺у埗妯″紡 鎵嬪姩/鑷姩
    mode = 1;  // 榛樿涓€寮€濮嬫槸鎵嬪姩

    while(1){
      Controller1.ButtonA.pressed([]()   
    {   
        mode = 1;
    });  // L1鏄墜鍔ㄦ帶鍒舵ā寮?

    Controller1.ButtonB.pressed([]()   
    {          
       // mode = 2;
    });   // L2鏄嚜鍔ㄦ帶鍒舵ā寮?
    return 0;
    }  
  }

  static void comp(){  // 姣旇禌鏃跺€欎娇鐢ㄧ殑鍑芥暟

    // 瀹炰緥鍖栨墜鍔ㄤ笌鑷姩鎺у埗绫?
    static ManualControl movecontroller;
    static AutoControl moveauto;

    imu.resetRotation();
    imu.resetHeading();
    // 妯″紡閫夋嫨绾跨▼
    vex::task modechoose(ModeChoose);  

    // 鎵嬪姩鎺у埗鐨勫簳鐩樼Щ鍔ㄧ嚎绋?
    vex::task manualcontrol(movecontroller.Manualcontrol); 

    // 鑷姩鎺у埗绾跨▼
    vex::task automove(moveauto.AutoMove); 

    double now_angle = imu.angle(deg);

    while(1){
      // 鑾峰緱褰撳墠杞
      now_angle = imu.angle(deg);
      Brain.Screen.clearLine();
      Brain.Screen.print("angle");
      Brain.Screen.print(now_angle);
      
      // 绾跨▼绠＄悊锛屾娴嬫槸鎵嬪姩杩樻槸鑷姩

      if(mode==2){  // 鑷姩鎺у埗鍒濆鍖?

        manualcontrol.suspend();
        // 閲嶆柊寮€濮嬩竴娈垫柊鐨勮嚜鍔ㄦ帶鍒剁嚎绋?
        automove = vex::task (moveauto.AutoMove); 
        // 闇€瑕佽烦杞埌mode3锛屽惁鍒欏皢浼氫竴鐩翠細鍦╩ode2涓噸澶嶅垵濮嬪寲鑷姩鎺у埗绾跨▼
        mode = 3;
        
      }else if(mode==1){  // 鎵嬪姩鎺у埗

        manualcontrol.resume();
        automove.suspend();
        
      }else{  // 鑷姩鎺у埗姝ｅ湪杩涜

        manualcontrol.suspend();

      }
    }
  }

};