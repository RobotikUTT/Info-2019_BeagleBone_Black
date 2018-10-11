/** @file GroupBalls.h
*    @brief Define GroupBalls class
*    
*    
*    @author Alexis CARE
*/
#ifndef GROUP_BALLS_H
#define GROUP_BALLS_H

#define _USE_MATH_DEFINES
#include "controller/OrientedPoint.h"
#include <math.h>
#include <cmath>

/**
 * @defgroup Ball The Ball action package
 * @{
 */

#define WALL_EPSILON 45 //mm
#define X_CANON 50 //mm
#define Y_CANON 145 //mm

/**
 * @brief      Class for group balls action type.
 */
class GroupBalls
{
public:
  OrientedPoint groupPoint;
  OrientedPoint shoot;
  OrientedPoint proc_point [4];
  int alpha;

  GroupBalls(int _xg, int _yg, int _rot_g, int _alpha, int x_delta, int _xs, int _ys, int _rot_s, bool side);
  GroupBalls();
private:

};

/**
 * @}
 */

#endif
