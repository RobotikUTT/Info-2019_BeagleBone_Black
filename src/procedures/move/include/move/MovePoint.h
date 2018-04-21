#ifndef MOVE_POINT_H
#define MOVE_POINT_H

#include <stdint.h>

class MovePoint
{
public:
  int16_t _x;
  int16_t _y;
  int16_t _angle;
  uint8_t _move_type;

  MovePoint();
  MovePoint(int16_t, int16_t);
  MovePoint(int16_t, int16_t, int16_t);
  MovePoint(int16_t, int16_t, int16_t, uint8_t);
};
#endif
