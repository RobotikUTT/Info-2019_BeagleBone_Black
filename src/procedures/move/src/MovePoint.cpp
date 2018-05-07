#include "move/MovePoint.h"


MovePoint::MovePoint():_x(0),_y(0),_angle(0), _move_type(0), _direction(0)
{
}

MovePoint::MovePoint(int16_t x, int16_t y): _x(x),_y(y),_angle(0), _move_type(1), _direction(0)
{
}

MovePoint::MovePoint(int16_t x, int16_t y, int16_t ang): _x(x),_y(y),_angle(ang), _move_type(0), _direction(0)
{
}

MovePoint::MovePoint(int16_t x, int16_t y, int16_t ang, uint8_t type): _x(x),_y(y),_angle(ang), _move_type(type), _direction(0)
{
}

MovePoint::MovePoint(int16_t x, int16_t y, int16_t ang, uint8_t type, int8_t dir): _x(x),_y(y),_angle(ang), _move_type(type), _direction(dir)
{
}
