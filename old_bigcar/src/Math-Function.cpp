// #include "tjulib/odom.hpp"
#include "math.h"
#include "Math-Functions.h"

using namespace tjulib;

const double PI = 3.14159265358979323846;

double Math::getWrap360(double currentAngle){
    while(currentAngle >= 360)
        currentAngle -= 360;
    while(currentAngle < 0)
        currentAngle += 360;
    return currentAngle;
}
double Math::getWrap2pi(double currentAngle){
    while(currentAngle >= 2 * PI)
        currentAngle -= 2 * PI;
    while(currentAngle < 0)
        currentAngle += 2 * PI;
    return currentAngle;
}


//CONVERSIONS (DISTANCE)
float Math::degToInch(float deg){
  return (deg / 360) * (M_PI *rad_In);
}
float Math::inchToDeg(float inch){
  return (inch / (M_PI * rad_In)) * 360;
}

//CONVERSIONS (ANGLE)
float Math::getRadians(float deg){
  return (deg * M_PI) / 180;
}
float Math::getDeg(float rad){
  return rad * (180/M_PI);
}

//HELPER FUNCTIONS
float Math::getHeading(float angle){
  while(!(angle >= 0 && angle < M_PI * 2)){
    if(angle < 0) angle += M_PI*2;
    if(angle >= M_PI * 2) angle -= M_PI * 2;
  }
  return angle;
}
//将任意角度值转换到 [-π, π] 的范围内，以解决角度的“跨越”问题
float Math::angleWrap(float angle){
  while(angle < -M_PI){
    angle += 2 * M_PI;
  }
  while(angle > M_PI){
    angle -= 2 * M_PI;
  }
  return angle;
}
//针对的是角度的“旋转”问题（也可以称之为“周期性”问题）
//假设有一个起始角度 startAngle，以及一个相对于该角度的偏移量 angle，那么该函数会将 angle 转换为 [startAngle-π×2, startAngle+π×2] 范围内的角度值
float Math::compressAngle(float startAngle, float angle){
  while(angle <= startAngle - M_PI*2){
    angle += M_PI*2;
  }
  while(angle >= startAngle + M_PI*2){
    angle -= M_PI*2;
  }
  return angle;
}

float Math::clip(float number, float min, float max){
  while(!(number >= min && number <= max)){
    if(number < min){
      number = min;
    }
    if(number > max){
      number = max;
    }
  }
  return number;
}
//确定机器人在旋转时选择最优的旋转方向
//当前朝向 currentA 和目标朝向 targetA，并返回整数值 1 或 -1，分别表示顺时针和逆时针旋转方向
int Math::optimalTurnSide(float currentA, float targetA){
  float diff = targetA - currentA;
  if(diff < 0){
    diff += M_PI*2;
  }
  if(diff > M_PI){
    return 1;
  } else {
    return -1;
  }
}

//GEOMETRY FUNCTIONS
//计算两个二维平面上点的距离
float Math::dist(graphPoint point1, graphPoint point2){
  return sqrt(pow(point2.x-point1.x, 2) + pow(point2.y-point1.y, 2));
}
//判断给定点 (point) 是否在给定的线段 (linePoint1 和 linePoint2 所确定) 上
bool Math::linePoint(graphPoint linePoint1, graphPoint linePoint2, graphPoint point) {
  // get distance from the point to the two ends of the line
  float d1 = dist({point.x, point.y}, {linePoint1.x,linePoint1.y});
  float d2 = dist({point.x, point.y}, {linePoint2.x,linePoint2.y});
  // get the length of the line
  float lineLen = dist({linePoint1.x,linePoint1.y}, {linePoint2.x,linePoint2.y});
  // since floats are so minutely accurate, add
  // a little buffer zone that will give collision
  float buffer = 0.1;    // higher # = less accurate
  // if the two distances are equal to the line's 
  // length, the point is on the line!
  // note we use the buffer here to give a range, 
  // rather than one #
  if (d1+d2 >= lineLen-buffer && d1+d2 <= lineLen+buffer) {
    return true;
  }
  return false;
}
//判断给定的点 (point) 是否在给定的圆 (circleCenter 和 cr 所确定) 内部
bool Math::pointCircle(graphPoint point, graphPoint circleCenter, float cr){
  if(dist({point.x, point.y}, {circleCenter.x, circleCenter.y}) < cr){
    return true;
  } else {
    return false;
  }
}
//判断给定的线段 (linePoint1 和 linePoint2 所确定) 是否与给定的圆 (circleCenter 和 r) 相交
bool Math::lineCircle(graphPoint linePoint1, graphPoint linePoint2, graphPoint circleCenter, float r) {

  // is either end INSIDE the circle?
  // if so, return true immediately
  bool inside1 = pointCircle({linePoint1.x, linePoint1.y}, {circleCenter.x, circleCenter.y}, r);
  bool inside2 = pointCircle({linePoint2.x, linePoint2.y}, {circleCenter.x, circleCenter.y},r);
  if (inside1 || inside2) return true;

  // get length of the line
  float distX = linePoint1.x - linePoint2.x;
  float distY = linePoint1.y - linePoint2.y;
  float len = sqrt( (distX*distX) + (distY*distY) );

  // get dot product of the line and circle
  float dot = ( ((circleCenter.x-linePoint1.x)*(linePoint2.x-linePoint1.x)) + 
                 ((circleCenter.y-linePoint1.y)*(linePoint2.y-linePoint1.y)) ) / pow(len,2);

  // find the closest point on the line
  float closestX = linePoint1.x + (dot * (linePoint2.x-linePoint1.x));
  float closestY = linePoint1.y + (dot * (linePoint2.y-linePoint1.y));

  // is this point actually on the line segment?
  // if so keep going, but if not, return false
  bool onSegment = linePoint({linePoint1.x, linePoint1.y}, {linePoint2.x, linePoint2.y}, {closestX, closestY});
  if (!onSegment) return false;

  // get distance to closest point
  distX = closestX - circleCenter.x;
  distY = closestY - circleCenter.y;
  float distance = sqrt( (distX*distX) + (distY*distY) );

  if (distance <= r) {
    return true;
  }
  return false;
}

//PURE PURSUIT
// std::vector<graphPoint> Math::lineCircleIntersection(graphPoint circleCenter, float radius, graphPoint linePoint1, graphPoint linePoint2){
//   //Convert line to slope intercept form
//   float slope = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
//   float yIntercept = linePoint1.y - (slope * linePoint1.x); //might not need
//   //Calculate the line perpendicular to the line being intersected which also crosses circle center
//   float newSlope = -1 / slope;
// }