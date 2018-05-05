#ifndef GROUP_BALLS_H
#define GROUP_BALLS_H

#define _USE_MATH_DEFINES
#include "controller/OrientedPoint.h"
#include <math.h>
#include <cmath>

#define WALL_EPSILON 45 //mm
#define X_CANON 50 //mm
#define Y_CANON 145 //mm
// #define EPSILON 15 //mm


class GroupBalls
{
public:

  GroupBalls(int _xg, int _yg, int _rot_g, int _alpha, int y_delta, int _xs, int _ys, int _rot_s, bool side);
  GroupBalls();

  OrientedPoint groupPoint;
  OrientedPoint shoot;
  int alpha;
  OrientedPoint proc_point [4];

private:

};
#endif
