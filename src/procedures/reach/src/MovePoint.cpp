/** @file MovePoint.cpp
*    @brief Move Point class
*    
*/
#include "reach/MovePoint.h"


/**
 * @brief      Constructs the default object.
 */
MovePoint::MovePoint():_x(0),_y(0),_angle(0), _move_type(0), _direction(0), _timeout(0)
{
}

/**
 * @brief      Constructs the object.
 *
 * @param[in]  x     End X Pos
 * @param[in]  y     End Y Pox
 */
MovePoint::MovePoint(int16_t x, int16_t y): _x(x),_y(y),_angle(0), _move_type(1), _direction(0), _timeout(0)
{
}

/**
 * @brief      Constructs the object.
 *
 * @param[in]  x     End X Pos
 * @param[in]  y     End Y Pos
 * @param[in]  ang   The end angle
 */
MovePoint::MovePoint(int16_t x, int16_t y, int16_t ang): _x(x),_y(y),_angle(ang), _move_type(0), _direction(0), _timeout(0)
{
}

/**
 * @brief      Constructs the object.
 *
 * @param[in]  x     End X Pos
 * @param[in]  y     End Y Pos
 * @param[in]  ang   The end angle
 * @param[in]  type  The Move type
 */
MovePoint::MovePoint(int16_t x, int16_t y, int16_t ang, uint8_t type): _x(x),_y(y),_angle(ang), _move_type(type), _direction(0), _timeout(0)
{
}

/**
 * @brief      Constructs the object.
 *
 * @param[in]  x     End X Pos
 * @param[in]  y     End Y Pos
 * @param[in]  ang   The end angle
 * @param[in]  type  The Move type
 * @param[in]  dir   The direction
 * @param[in]  time  The timeout of the order
 */
MovePoint::MovePoint(int16_t x, int16_t y, int16_t ang, uint8_t type, int8_t dir, uint16_t time): _x(x),_y(y),_angle(ang), _move_type(type), _direction(dir), _timeout(time)
{
}
