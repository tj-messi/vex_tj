#pragma once
#include <vector>

namespace tjulib{

  struct graphPoint
  {
    float x, y;
  };

  class Math
  {
    private:
      const float rad_In;
    public:
      // CONVERSIONS (DISTANCE)
      Math(float rad_In):rad_In(rad_In){};
      float degToInch(float deg);
      float inchToDeg(float inch);

      // CONVERSIONS (ANGLE)
      static float getRadians(float deg);
      static float getDeg(float rad);
      static double getWrap2pi(double currentAngle);
      static double getWrap360(double currentAngle);

      // HELPER FUNCTIONS
      static float getHeading(float angle);
      static float compressAngle(float startAngle, float angle);
      static float clip(float number, float min, float max);

      // GEOMETRY FUNCTIONS
      static float dist(graphPoint point1, graphPoint point2);
      static bool linePoint(graphPoint linePoint1, graphPoint linePoint2, graphPoint point);
      static bool pointCircle(graphPoint point, graphPoint circleCenter, float cr);
      static bool lineCircle(graphPoint linePoint1, graphPoint linePoint2, graphPoint circleCenter, float r);

      static constexpr float velocityToVoltage = 12000/200;


      // PURE PURSUIT
      std::vector<graphPoint> lineCircleIntersection(graphPoint circleCenter, float radius,
                                                            graphPoint linePoint1, graphPoint linePoint2);
  };
};

