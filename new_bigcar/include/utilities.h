#pragma once
// 方向cur、tar[0,360)，将cur方向移动到tar方向，
// 就近选择移动方式，顺时针为正逆时针为负，返回值的大小为需要旋转的度数
double shortestDiff(double cur, double tar);
double warpAngle_deg(double angle);
double getAngle(double x, double y);
double KeepInRange(double x, const double min, const double max);
double KeepInRange_abs(double x, const double min, const double max);