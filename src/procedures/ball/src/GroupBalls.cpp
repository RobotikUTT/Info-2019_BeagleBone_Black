#include "ball/GroupBalls.h"

GroupBalls::GroupBalls(int _xg, int _yg, int _rot_g, int _alpha, int y_delta, int _xs, int _ys, int _rot_s, bool side){
  groupPoint = OrientedPoint(_xg,_yg);
  alpha = _alpha;

  if (side){ //side orange
    shoot = OrientedPoint(_xs,_ys,_rot_s + 1571);
    proc_point[0] = OrientedPoint(_xg + std::tan(alpha)*-y_delta , _yg + y_delta, - alpha + 3141);
    proc_point[1] = OrientedPoint(_xg - Y_CANON - WALL_EPSILON , _yg + X_CANON, 3141 );

  } else {
    shoot = OrientedPoint(_xs,_ys,_rot_s);
    proc_point[0] = OrientedPoint(_xg + std::tan(alpha)*y_delta , _yg - y_delta, - alpha);
    proc_point[1] = OrientedPoint(_xg + Y_CANON + WALL_EPSILON , _yg - X_CANON, 0 );

  }

}

GroupBalls::GroupBalls(){}
