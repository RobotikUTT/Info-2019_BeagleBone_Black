#include "scheduler/ActionClass.h"

//  Constructors
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

ActionClass::ActionClass(std::string name, int8_t action, ActionPoint PAction , int point, float fact){
  this->_priority = 0;
  this->_name = name;
  this->_action = action;
  this->_done = false;
  this->_possible = true;
  this->_value = point;
  this->PAction = PAction;
  this->_diff_factor = fact;
}

// Methodes

void ActionClass::updatePriority(Point& robot_pos) {
  if(!this->_done){
    int temp = PAction.startPoint.manhattanDist(robot_pos) / 100;
    if (PAction.endPoint.x >= 0 ) {
      temp += PAction.startPoint.manhattanDist(PAction.endPoint)/100;
    }
    temp +=  _value / _diff_factor;
    this->_priority = temp;
  }
}

void ActionClass::checkPossibility() {
  /* code */
}

void ActionClass::changeSide() {
  PAction.startPoint.x = (1500 - PAction.startPoint.x) + 1500;
  if (PAction.endPoint.x >= 0 ) {
    PAction.endPoint.x = (1500 - PAction.endPoint.x) + 1500;
  }
}


bool ActionClass::operator< ( const ActionClass &B)const {
  if (_done) return 0;
  return (_priority < B._priority);
}

std::ostream& operator<<(std::ostream& os, const ActionClass& AC)
{
    os << "ActionClass : { \n name: " << AC._name << "\n value: " << AC._value << "\n priority: " << AC._priority << "\n done: " << AC._done << "\n possible: " << AC._possible << "\n " << AC.PAction << std::endl;
    return os;
}


// 3000 largeur x
// 2000 longueur y

// cote vert standart
