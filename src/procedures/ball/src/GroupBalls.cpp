#include "ball/GroupBalls.h"

GroupBalls::GroupBalls(int _xg, int _yg, int _rot_g, int _alpha, int x_delta, int _xs, int _ys, int _rot_s, bool side){
  groupPoint = OrientedPoint(_xg,_yg);
  alpha = _alpha;

  if (side){ //side orange
    int new_angle = 3141 - _alpha;
    shoot = OrientedPoint(_xs,_ys, new_angle);
    proc_point[0] = OrientedPoint(_xg + x_delta , _yg + std::tan(alpha)*-x_delta, _rot_g - new_angle );
    proc_point[1] = OrientedPoint(_xg + X_CANON , _yg - Y_CANON - WALL_EPSILON, _rot_g );

  } else {
    shoot = OrientedPoint(_xs,_ys,_rot_s);
    proc_point[0] = OrientedPoint(_xg - x_delta , _yg + std::tan(alpha)*x_delta, _rot_g + alpha);
    proc_point[1] = OrientedPoint(_xg - X_CANON , _yg + Y_CANON + WALL_EPSILON , _rot_g );
  }
}

GroupBalls::GroupBalls(){}
