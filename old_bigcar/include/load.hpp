#pragma once
#include "vex.h"

class Load{
    #define MIDDLE 0
    #define LEFT -40
    #define RIGHT 45
    #define HALFLEFT -30
private:
    vex::motor* m;

public:
    Load(vex::motor* m) : m(m){
        // init position is RIGHT
        m->setPosition(RIGHT, deg);
        m->setBrake(vex::brakeType::hold);
        
    }
    void setPosition(double x){m->setPosition(x, deg);}
    void show(){printf("deg: %.2f\n", m->position(deg));}
    // V is always positive
    void kickLeft(double V = 70){
        m->spinTo(LEFT, deg, -V, vex::velocityUnits::pct);
        task::sleep(50);
    }
    void kickRight(double V = 70){
        m->spinTo(RIGHT, deg, V, vex::velocityUnits::pct);
        task::sleep(50);
    }
    void holdForPush(double V = 70){
        m->spinTo(HALFLEFT, deg, V, vex::velocityUnits::pct);
        m->setBrake(vex::brakeType::hold);
        task::sleep(50);

    }
    void continuousKick(const int times, const int gapmTime = 1000){
        for(int i=0;i<times;i++){  
            kickLeft();  
            task::sleep(gapmTime);
            kickRight();    
        }
        kickRight(40); 
    }
    // spin extra and hold
    void hold(){
        m->spin(vex::directionType::fwd, 100, vex::pct);
        task::sleep(300);
        m->stop(vex::brakeType::hold);
    }

};