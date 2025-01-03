#pragma once
#include "vex.h"
#include "tjulib-uppercontrol/ConveyRing.hpp"
#include "tjulib-uppercontrol/LiftArm.hpp"
#include "tjulib-chassis/oct-chassis/oct-cur.hpp"
#include "tjulib-motionplanner/tjulib-pathplanner/rrt.hpp"
#include "tjulib-map/HighStakeMap.hpp"
#include <string>
#include <iostream>
using namespace vex;
typedef double T;

namespace tjulib
{
    class Oct_Action : virtual public Oct_CurChassis{
    private:
        const double PI = 3.14159265358979323846;
        
    public:
        RRT *rrt = NULL;
        Oct_Action(std::vector<std::vector<vex::motor*>*>& _chassisMotors, pidControl* _motorpidControl, Position* _position, const T _r_motor, pidControl* _fwdpid, pidControl* _turnpid, PurePursuit *_ppcontrol, RRT *rrt) :
            Oct_BaseChassis(_chassisMotors, _motorpidControl, _position, _r_motor), 
            Oct_StraChassis(_chassisMotors, _motorpidControl, _position, _r_motor, _fwdpid, _turnpid),
            Oct_CurChassis(_chassisMotors, _motorpidControl, _position, _r_motor, _fwdpid, _turnpid, _ppcontrol),
            rrt(rrt) {}
        

        void ActionChoose(int action_index, Point target, T goal_pt_track_time = 100, T maxSpeed = 100, T maxtime_ms = 15000, T move_gaptime = 10 ,int fwd = 1, int back = 0){
            switch(action_index){
             case 0:
                back = 0;   // 正着取环
                MoveForRing(target, goal_pt_track_time, maxSpeed, maxtime_ms, move_gaptime ,fwd, back);
                break;
            case 1:
                back = 1;   // 倒着取桩
                MoveForStake(target, goal_pt_track_time, maxSpeed, maxtime_ms, move_gaptime ,fwd, back);
                break;
            case 2:         // 倒着放桩
                back = 1;
                LayDownStake(target, goal_pt_track_time, maxSpeed, maxtime_ms, move_gaptime ,fwd, back);
                break;
            case 3:         // 倒着扣环
                back = 1;
                SlamDownRing(target, goal_pt_track_time, maxSpeed, maxtime_ms, move_gaptime ,fwd, back);
                break;
            case 4:         // 正着取环(取一半)
                back = 0;
                MoveForRingHalf(target, goal_pt_track_time, maxSpeed, maxtime_ms, move_gaptime ,fwd, back);
                break;
            default:
                break;
            }
           
        }


        // 取环
        void MoveForRing(Point target, T goal_pt_track_time = 100, T maxSpeed = 100, T maxtime_ms = 15000, T move_gaptime = 10 ,int fwd = 1, int back = 0){
            
            /*= Step1 : 如果机器人在操作半径外则需要先到达可执行的操作半径范围内 =*/
            const T radius = 10; 
            if(fabs(GetDistance(position->globalPoint, target)) >= radius){  

                /*= Step1.1 : 计算轨迹与圆的交点 =*/
                Point target_pt = CalcuTargetPoint(target, radius);
                printf("x : %lf, y : %lf \n", target_pt.x, target_pt.y);
                /*= Step1.2 : RRT规划器获取轨迹 =*/
                printf("rrt_start\n");   
                printf("x0 : %lf, y0 : %lf \n", gps_x, gps_y);
                printf("x : %lf, y : %lf \n", target_pt.x, target_pt.y);
                std::vector<Point> path = rrt->optimal_rrt_planning((Point){gps_x, gps_y}, (Point){target_pt.x, target_pt.y});  
                for(auto point : path){
                    printf("{x:%lf , y:%lf}\n", point.x, point.y);
                }
                printf("rrt_end\n");
            
                /*= Step1.3 : 规划路径跟踪 =*/
                PathMove(path, goal_pt_track_time, maxSpeed, maxtime_ms, move_gaptime, fwd, back);
            }
            
            /*= Step2 :执行最后一步动作 =*/
            turnToTarget(target,  maxSpeed, maxtime_ms,  fwd, back);
            // 开启吸环线程
            manual = false;
            ring_convey_spin = true;
            reinforce_stop = false;
            moveToTarget(target, maxSpeed, maxtime_ms, move_gaptime, fwd);
            

        }

        // 取环
        void MoveForRingHalf(Point target, T goal_pt_track_time = 100, T maxSpeed = 100, T maxtime_ms = 15000, T move_gaptime = 10 ,int fwd = 1, int back = 0){
            
            /*= Step1 : 如果机器人在操作半径外则需要先到达可执行的操作半径范围内 =*/
            const T radius = 15; 
            if(fabs(GetDistance(position->globalPoint, target)) >= radius){  

                /*= Step1.1 : 计算轨迹与圆的交点 =*/
                Point target_pt = CalcuTargetPoint(target, radius);

                /*= Step1.2 : RRT规划器获取轨迹 =*/
                printf("rrt_start\n");   
                std::vector<Point> path = rrt->optimal_rrt_planning((Point){position->globalPoint.x, position->globalPoint.y}, (Point)(target_pt), 4);  
                // for(auto point : path){
                //     printf("{x:%lf , y:%lf}\n", point.x, point.y);
                // }
                printf("rrt_end\n");
            
                /*= Step1.3 : 规划路径跟踪 =*/
                PathMove(path, goal_pt_track_time, maxSpeed, maxtime_ms, move_gaptime, fwd, back);
            }
            
            /*= Step2 :执行最后一步动作 =*/
            turnToTarget(target,  maxSpeed, maxtime_ms, fwd, back);
            // 开启半吸环线程
            manual = false;
            ring_convey_spin = true;
            reinforce_stop = false;
            half_ring_get = true;
            moveToTarget(target, maxSpeed, maxtime_ms, move_gaptime, fwd);
            half_ring_get = false;
        }

        // 取桩
        void MoveForStake(Point target, T goal_pt_track_time = 100, T maxSpeed = 100, T maxtime_ms = 15000, T move_gaptime = 10 ,int fwd = 1, int back = 1){
           /*= Step1 : 如果机器人在操作半径外则需要先到达可执行的操作半径范围内 =*/
            const T radius = 20; 
            if(fabs(GetDistance(position->globalPoint, target)) >= radius){  

                /*= Step1.1 : 计算轨迹与圆的交点 =*/
                Point target_pt = CalcuTargetPoint(target, radius);

                /*= Step1.2 : RRT规划器获取轨迹 =*/
                printf("rrt_start\n");   
                std::vector<Point> path = rrt->rrt_planning({position->globalPoint.x, position->globalPoint.y}, (Point)(target_pt));  
                // for(auto point : path){
                //     printf("{x:%lf , y:%lf}\n", point.x, point.y);
                // }
                printf("rrt_end\n");
            
                /*= Step1.3 : 规划路径跟踪 =*/
                PathMove(path, goal_pt_track_time, maxSpeed, maxtime_ms, move_gaptime, fwd, back);
            }
            
            /*= Step2 :执行最后一步动作 =*/
            turnToTarget(target,  maxSpeed, maxtime_ms,  fwd, back);
            moveToTarget(target, maxSpeed, maxtime_ms, move_gaptime, fwd);
            gas_hold.state(100, pct);

        }

        // 将桩放置到得分区(这里由于实际上得分区在四角，不考虑一般情况，只对特定的四角做讨论处理即可)
        void LayDownStake(Point target, T goal_pt_track_time = 100, T maxSpeed = 100, T maxtime_ms = 15000, T move_gaptime = 10 ,int fwd = 1, int back = 1){
            /*= Step1 : 如果机器人在操作半径外则需要先到达可执行的操作半径范围内 =*/
            const T radius = 20; 
            const T target_ = 59;
            const T step_back = 50;
            const T corner = 72;
            if(fabs(GetDistance(position->globalPoint, target)) >= radius){  

                /*= Step1.1 : 计算轨迹与圆的交点 =*/
                Point target_pt = CalcuTargetPoint(target, radius);
                target_pt.angle = 0;
                /*= Step1.2 : RRT规划器获取轨迹 =*/
                printf("rrt_start\n");   
                std::vector<Point> path = rrt->optimal_rrt_planning({position->globalPoint.x, position->globalPoint.y, 0}, (Point)(target_pt), 4);  
                // for(auto point : path){
                //     printf("{x:%lf , y:%lf}\n", point.x, point.y);
                // }
                printf("rrt_end\n");

                /*= Step1.3 : 规划路径跟踪 =*/
                PathMove(path, goal_pt_track_time, maxSpeed, maxtime_ms, move_gaptime, fwd, back);

                /*= Step2 : 放桩 : 正常只需要对齐目标位置，把桩推进去即可放下再出来即可 =*/
                if(target.x == target_ && target.y == target_){
                    turnToTarget({corner, corner},  maxSpeed, maxtime_ms,  fwd, back);
                    moveToTarget({target_, target_}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                    gas_hold.state(0, pct);
                    task::sleep(300);
                    moveToTarget({step_back, step_back}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                }else if(target.x == -target_ && target.y == target_){
                    turnToTarget({-corner, corner},  maxSpeed, maxtime_ms,  fwd, back);
                    moveToTarget({-target_, target_}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                    gas_hold.state(0, pct);
                    task::sleep(300);
                    moveToTarget({-step_back, step_back}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                }else if(target.x == target_ && target.y == -target_){
                    turnToTarget({corner, -corner},  maxSpeed, maxtime_ms,  fwd, back);
                    moveToTarget({target_, -target_}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                    gas_hold.state(0, pct);
                    task::sleep(300);
                    moveToTarget({step_back, -step_back}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                }else{
                    turnToTarget({-corner, -corner},  maxSpeed, maxtime_ms,  fwd, back);
                    moveToTarget({-target_, -target_}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                    gas_hold.state(0, pct);
                    task::sleep(300);
                    moveToTarget({-step_back, -step_back}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                }

            }else{      // 如果一开始就在半径内，则需要先退后出来，再转向、放桩
                if(target.x == target_ && target.y == target_){
                    moveToTarget({step_back, step_back}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                    turnToTarget({corner, corner},  maxSpeed, maxtime_ms,  fwd, back);
                    moveToTarget({target_, target_}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                    gas_hold.state(0, pct);
                    task::sleep(300);
                    moveToTarget({step_back, step_back}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                }else if(target.x == -target_ && target.y == target_){
                    moveToTarget({-step_back, step_back}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                    turnToTarget({-corner, corner},  maxSpeed, maxtime_ms,  fwd, back);
                    moveToTarget({-target_, target_}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                    gas_hold.state(0, pct);
                    task::sleep(300);
                    moveToTarget({-step_back, step_back}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                }else if(target.x == target_ && target.y == -target_){
                    moveToTarget({step_back, -step_back}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                    turnToTarget({corner, -corner},  maxSpeed, maxtime_ms,  fwd, back);
                    moveToTarget({target_, -target_}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                    gas_hold.state(0, pct);
                    task::sleep(300);
                    moveToTarget({step_back, -step_back}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                }else{
                    moveToTarget({-step_back, -step_back}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                    turnToTarget({-corner, -corner},  maxSpeed, maxtime_ms,  fwd, back);
                    moveToTarget({-target_, -target_}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                    gas_hold.state(0, pct);
                    task::sleep(300);
                    moveToTarget({-step_back, -step_back}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                }
            }
            

        }

        // 将环扣到固定桩(这里由于实际上得分区在四边，不考虑一般情况，只对特定的四边做讨论处理即可)
        void SlamDownRing(Point target, T goal_pt_track_time = 100, T maxSpeed = 100, T maxtime_ms = 15000, T move_gaptime = 10 ,int fwd = 1, int back = 1){
            /*= Step1 : 如果机器人在操作半径外则需要先到达可执行的操作半径范围内 =*/
            const T radius = 20; 
            const T target_ = 60;
            const T side = 72;
            if(fabs(GetDistance(position->globalPoint, target)) >= radius){  

                /*= Step1.1 : 计算轨迹与圆的交点 =*/
                Point target_pt = CalcuTargetPoint(target, radius);
            
                /*= Step1.2 : RRT规划器获取轨迹 =*/
                printf("rrt_start\n");   
                std::vector<Point> path = rrt->optimal_rrt_planning({position->globalPoint.x, position->globalPoint.y}, (Point)(target_pt), 4);  
                // for(auto point : path){
                //     printf("{x:%lf , y:%lf}\n", point.x, point.y);
                // }
                printf("rrt_end\n");
            
                /*= Step1.3 : 规划路径跟踪 =*/
                PathMove(path, goal_pt_track_time, maxSpeed, maxtime_ms, move_gaptime, fwd, back);
            }
            if(target.x == 0 && target.y == target_){
                moveToTarget({0, target_}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                turnToTarget({0, side},  maxSpeed, maxtime_ms,  fwd, back);
                // 放环
                thread arm_lift(Arm_Lift);
                thread arm_down(Arm_Down);
            }else if(target.x == target_ && target.y == 0){
                moveToTarget({target_, 0}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                turnToTarget({side, 0},  maxSpeed, maxtime_ms,  fwd, back);
                // 放环
                thread arm_lift(Arm_Lift);
                thread arm_down(Arm_Down);
            }else if(target.x == 0 && target.y == -target_){
                moveToTarget({0, -target_}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                turnToTarget({0, -side},  maxSpeed, maxtime_ms,  fwd, back);
                // 放环
                thread arm_lift(Arm_Lift);
                thread arm_down(Arm_Down);
            }else{
                moveToTarget({-target_, 0}, maxSpeed, maxtime_ms, move_gaptime, fwd);
                turnToTarget({-side, 0},  maxSpeed, maxtime_ms,  fwd, back);
                // 放环
                thread arm_lift(Arm_Lift);
                thread arm_down(Arm_Down);
            }
        }

        // 借助一次粗规划求解目标点
        Point CalcuTargetPoint(Point target, T radius){
            // 误差允值是2倍步长
            T error = 2 * rrt->step;

            // 先进行一次粗规划
            Point start_pt = {position->globalPoint.x, position->globalPoint.y};
            std::vector<Point> path = rrt->rrt_planning(start_pt, (Point)(target));  // 这里一定要强制类型转换为Point
            for(auto point : path){
                    printf("{x:%lf , y:%lf}\n", point.x, point.y);
            }
            // 二分法计算坐标
            int left = 0;
            int right = path.size() - 1;
            T distance = -1;
            while(fabs(distance - radius) >= error){

                int middle = (left + right) / 2;
                
                distance = sqrt((path[middle].x - target.x) * (path[middle].x - target.x) + (path[middle].y - target.y) * (path[middle].y - target.y));
                
                printf("left:%d right:%d middle:%d distance : %lf \n",left, right, middle, distance);
                
                // 计算距离
                if(distance < radius){
                    // 从期望上讲，如果能够满足单调关系，则新middle会比老middle更大，否则会陷入死循环
                    right = middle;
                    // 对死循环困境进行处理，如果出现困境则强制right++变得更大
                    T dis_nxt = sqrt((path[(left + right) / 2].x - target.x) * (path[(left + right) / 2].x - target.x) + (path[(left + right) / 2].y - target.y) * (path[(left + right) / 2].y - target.y));
                    while(dis_nxt < distance){
                        right++;
                        dis_nxt = sqrt((path[(left + right) / 2].x - target.x) * (path[(left + right) / 2].x - target.x) + (path[(left + right) / 2].y - target.y) * (path[(left + right) / 2].y - target.y));
                    }
                    
                }else{
                    // 从期望上讲，如果能够满足单调关系，则新middle会比老middle更小，否则会陷入死循环
                    left = middle;
                    // 对死循环困境进行处理，如果出现困境则强制left--变得更小
                    T dis_nxt = sqrt((path[(left + right) / 2].x - target.x) * (path[(left + right) / 2].x - target.x) + (path[(left + right) / 2].y - target.y) * (path[(left + right) / 2].y - target.y));
                    while(dis_nxt > distance){
                        left--;
                        dis_nxt = sqrt((path[(left + right) / 2].x - target.x) * (path[(left + right) / 2].x - target.x) + (path[(left + right) / 2].y - target.y) * (path[(left + right) / 2].y - target.y));
                    }
                }
            }

            return path[(left + right) / 2];
            
        }

    };
}