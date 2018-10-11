/** @file GroupBlocks.h
*    @brief Define GroupBlocks class
*    
*    
*    @author Alexis CARE
*/
#ifndef GROUP_BLOCK_H
#define GROUP_BLOCK_H

#define _USE_MATH_DEFINES
#include "controller/OrientedPoint.h"
#include <math.h>
#include <cmath>
#include "ros/ros.h"

/**
 * @defgroup Block The block action package
 * @{
 */

#define SEMI_CUBE_LENTH 29 //mm
#define RADIUS_ROBOT 170 //mm
#define CENTER_PLIERS 113 //mm
#define EPSILON 50 //mm

/**
 * @brief      Class for group blocks action.
 */
class GroupBlocks
{
public:

  GroupBlocks(int _xg, int _yg, int _rot, int _xd, int _yd, bool side);
  GroupBlocks();

  OrientedPoint groupPoint;
  OrientedPoint depos;
  OrientedPoint depos_;
  int rot;
  OrientedPoint proc_point [10];

private:

};

/**
 * @}
 */

#endif
