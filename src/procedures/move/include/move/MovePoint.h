/** @file MovePoint.h
*    @brief Define MovePoint class
*    
*    @author Alexis CARE
*/
#ifndef MOVE_POINT_H
#define MOVE_POINT_H

#include <stdint.h>

/**
 * @defgroup move The move package
 * @{
 */

/**
 * @brief      Class for move point.
 */
class MovePoint
{
public:
  int16_t _x;
  int16_t _y;
  int16_t _angle;
  uint8_t _move_type;
  int8_t _direction;
  uint16_t _timeout;

  MovePoint();
  MovePoint(int16_t, int16_t);
  MovePoint(int16_t, int16_t, int16_t);
  MovePoint(int16_t, int16_t, int16_t, uint8_t);
  MovePoint(int16_t, int16_t, int16_t, uint8_t, int8_t, uint16_t);
};

/**
 * @}
 */
#endif
