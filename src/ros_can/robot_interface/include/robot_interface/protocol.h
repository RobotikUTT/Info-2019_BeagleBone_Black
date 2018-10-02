/** @file protocol.h
*    @brief File wich defined the CAN Protocol message type
*    
*/

/**
 * @defgroup Robot_Interface The robot_interface package
 * @{
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#define ALL_CAN_ADDR            0
#define BBB_CAN_ADDR            1
#define STM_CAN_ADDR            2
#define ARDUINO_CAN_ADDR        3
#define ZIGBEE_CAN_ADDR         4
#define PANEL_CAN_ADDR          5

#define STOP                    0
#define START                   1
#define PAUSE                   2
#define RESUME                  3
#define RESET_ID                4
#define SETEMERGENCYSTOP        5
#define NEXT_ORDER              6
#define RESET_ORDERS            7
#define UNSETEMERGENCYSTOP      8


#define HANDSHAKE               0
#define WHOAMI                  1
#define SET_MODE                2

#define SPD                     4
#define GET_CODER               5
#define SEND_POINT              6
#define MANAGEMENT              7
#define GOTOA                   8
#define GOTO                    9
#define ROT                     10
#define ROTNOMODULO             11
#define PIDLEFT                 12
#define PIDRIGHT                13
#define PIDALL                  14
#define PWM                     15
#define SET_POS                 16
#define SET_PARAM               17
#define CURRENT_POS             18
#define CURRENT_PWM             19
#define CURRENT_SPD             20
#define MOVE_PLIERS             21
#define CLOSE_OPEN_PLIERS       22
#define SONAR_DISTANCE          23
#define THROW_BALLS             24
#define OBJECT_ON_MAP           25
#define ORDER_COMPLETED         26
#define SET_SERVO               27
#define ROBOT_BLOCKED           28
#define ACTION_PLIERS           29

/**
 * @}
 */

#endif
