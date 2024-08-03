/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\dwc240327                                        */
/*    Created:      Fri Jun 28 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Left1                motor_group   3, 2            
// Left2                motor_group   4, 5            
// Right1               motor_group   7, 6            
// Right2               motor_group   8, 9            
// imu                  inertial      11              
// arm1                 motor_group   18, 1           
// clip                 motor_group   14, 15          
// arm2                 motor_group   10, 19          
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "Competition.cpp"
using namespace vex;


int main() {

  Comp comp;
  comp.comp();

}