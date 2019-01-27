/** @file ActionClass.cpp
*    @brief Class for ActionClass.
*    
*    
*    @author Alexis CARE
*/
#include "scheduler/Action.hpp"

/**
 * @brief      Constructs the default object.
 */
Action::Action(std::string name){
  this->_name = name;
  this->_state = ACTION_IDLE;
}

/**
 * @brief      Return the priority of the action according to it's points and distance
 *
 * @param      robot_pos  current robot position
 */
int Action::priority(Point& robot_pos) {
  int action_points = points();

  return action_points * action_points / startPoint().manhattanDist(robot_pos);
}


/*void ActionClass::changeSide(){
  PAction.startPoint.y = (1500 - PAction.startPoint.y) + 1500;
  PAction.startPoint.angle *= -1;
  if (PAction.endPoint.y >= 0 ) {
    PAction.endPoint.y = (1500 - PAction.endPoint.y) + 1500;
    PAction.endPoint.angle *= -1;
  }
}*/

/**
 * @brief      Operator '<' for two ActionClass
 *
 * @param[in]  B     Other ActionClass
 *
 * @return     This < Other
 */
/*bool Action::operator< ( const ActionClass &B)const{
  if (_done) return 0;
  return (_priority < B._priority);
}*/

