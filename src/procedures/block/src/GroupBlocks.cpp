/** @file GroupBlocks.cpp
*    @brief The Group Block class
*    
*    @author Alexis CARE
*/
#include "block/GroupBlocks.h"


/**
 * @brief      Signe function
 * 
 * @details    See : https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
 *
 * @param[in]  val   The value
 *
 * @tparam     T     Number type
 *
 * @return     returns -1, 0 or 1
 */
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

/**
 * @brief      Constructs the object.
 * 
 * @details    Allway take blocks clockwise
 *
 * @param[in]  _xg   Pos X of goal (center of blocks)
 * @param[in]  _yg   Pos Y of goal (center of blocks)
 * @param[in]  _rot  Where the first block will be picked
 * @param[in]  _xd   Pos X of depos
 * @param[in]  _yd   Pos Y of depos
 * @param[in]  side  The side (Green or Orange  )
 */
GroupBlocks::GroupBlocks(int _xg, int _yg, int _rot, int _xd, int _yd, bool side){
  groupPoint = OrientedPoint(_xg,_yg);
  rot = _rot;

  int D3 = 3*SEMI_CUBE_LENTH + RADIUS_ROBOT + EPSILON;
  int d3 = 3*SEMI_CUBE_LENTH + CENTER_PLIERS;
  int d1 = SEMI_CUBE_LENTH + CENTER_PLIERS;
  int d_1 = -SEMI_CUBE_LENTH + CENTER_PLIERS;

  depos = OrientedPoint(_xd + d1, _yd, -3141);
  depos_ = OrientedPoint(_xd  + D3,_yd, -3141);

  proc_point[0] = OrientedPoint(_xg + sgn(std::cos((-3*M_PI)/4 + (float)rot/1000))* D3, _yg + sgn(std::sin((-3*M_PI)/4 + (float)rot/1000)) * D3);
  proc_point[1] = OrientedPoint(_xg + (int)round(std::cos(M_PI + (float)rot/1000))* D3, _yg + (int)round(std::sin(M_PI + (float)rot/1000)) * D3, rot);
  proc_point[2] = OrientedPoint(_xg + (int)round(std::cos(M_PI + (float)rot/1000))* d3, _yg + (int)round(std::sin(M_PI + (float)rot/1000)) * d3, rot);
  proc_point[3] = OrientedPoint(_xg + (int)round(std::cos(-M_PI/2 + (float)rot/1000))* D3, _yg + (int)round(std::sin(-M_PI/2 + (float)rot/1000)) * D3, 1571 + rot);
  proc_point[4] = OrientedPoint(_xg + (int)round(std::cos(-M_PI/2 + (float)rot/1000))* d3, _yg + (int)round(std::sin(-M_PI/2 + (float)rot/1000)) * d3, 1571 + rot);
  proc_point[5] = OrientedPoint(_xg + sgn(std::cos(-M_PI/4 + (float)rot/1000))* D3, _yg + sgn(std::sin(-M_PI/4 + (float)rot/1000)) * D3);
  proc_point[6] = OrientedPoint(_xg + (int)round(std::cos(+(float)rot/1000))* D3, _yg + (int)round(std::sin(+(float)rot/1000)) * D3, 3141 + rot);
  proc_point[7] = OrientedPoint(_xg + (int)round(std::cos(+(float)rot/1000))* d3, _yg + (int)round(std::sin(+(float)rot/1000)) * d3, 3141 + rot);
  proc_point[8] = OrientedPoint(_xg + (int)round(std::cos((-M_PI)/2 + (float)rot/1000))* d1, _yg + (int)round(std::sin((-M_PI)/2 + (float)rot/1000)) * d1, 1571 + rot);
  proc_point[9] = OrientedPoint(_xg + (int)round(std::cos((-M_PI)/2 + (float)rot/1000))* d_1, _yg + (int)round(std::sin((-M_PI)/2 + (float)rot/1000)) * d_1, 1571 + rot);

  if (side) { //if orange
    for (int i = 0; i < 10; i++) {
      proc_point[i].y = (_yg - proc_point[i].y) + _yg;
      proc_point[i].angle = -proc_point[i].angle;
    }
  }

}

/**
 * @brief      Constructs the default object.
 */
GroupBlocks::GroupBlocks(){
}
