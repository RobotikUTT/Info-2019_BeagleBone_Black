/** @file ActionClass.cpp
*    @brief Class for ActionClass.
*    
*    
*    @author Alexis CARE
*/
#include "scheduler/ActionClass.h"

/**
 * @brief      Constructs the default object.
 */
ActionClass::ActionClass(){
  this->_priority = 0;
  this->_name = "";
  this->_action = -1;
  this->_done = false;
  this->_possible = true;
  this->_value = 0;
  this->PAction = ActionPoint();
  this->_diff_factor = 1;
}

/**
 * @brief      Constructs the object.
 *
 * @param[in]  name     The name
 * @param[in]  action   The action
 * @param[in]  PAction  The p action
 * @param[in]  point    The point
 * @param[in]  fact     The fact
 * @param[in]  para     The para
 */
ActionClass::ActionClass(std::string name, int8_t action, ActionPoint PAction , int point, float fact, std::vector<int> para){
  this->_priority = 0;
  this->_name = name;
  this->_action = action;
  this->_done = false;
  this->_possible = true;
  this->_value = point;
  this->PAction = PAction;
  this->_diff_factor = fact;
  this->_param = para;
}

// Methodes
//
// @param      robot_pos  The robot position
//
void ActionClass::updatePriority(Point& robot_pos){
  if(!this->_done){
    int temp = PAction.startPoint.manhattanDist(robot_pos) / 100;
    if (PAction.endPoint.x >= 0 ) {
      temp += PAction.startPoint.manhattanDist(PAction.endPoint)/100;
    }
    temp +=  _value * _diff_factor;
    this->_priority = temp;
  }
}

/**
 * @brief      Check if action is possible
 * 
 * @todo Dev
 */
void ActionClass::checkPossibility(){
  /* code */
}

/**
 * @brief      Change actionClass side
 */
void ActionClass::changeSide(){
  PAction.startPoint.y = (1500 - PAction.startPoint.y) + 1500;
  PAction.startPoint.angle *= -1;
  if (PAction.endPoint.y >= 0 ) {
    PAction.endPoint.y = (1500 - PAction.endPoint.y) + 1500;
    PAction.endPoint.angle *= -1;
  }
}

/**
 * @brief      Operator '<' for two ActionClass
 *
 * @param[in]  B     Other ActionClass
 *
 * @return     This < Other
 */
bool ActionClass::operator< ( const ActionClass &B)const{
  if (_done) return 0;
  return (_priority < B._priority);
}

/**
 * @brief      Transform an ActionClass to a stream
 *
 * @param      os    The operating system
 * @param[in]  AC    A ActionClass
 * 
 * @return     A stream
 */
std::ostream& operator<<(std::ostream& os, const ActionClass& AC){
    os << "ActionClass : { \n name: " << AC._name << "\n value: " << AC._value << "\n priority: " << AC._priority << "\n done: " << AC._done << "\n possible: " << AC._possible << "\n " << AC.PAction << std::endl;
    return os;
}
// cote vert standart
