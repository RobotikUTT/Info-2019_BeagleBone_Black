#include "ActionClass.h"

//  Constructors
ActionClass::ActionClass(){
  this->_priority = 0;
  this->_name = "";
  this->_done = false;
  this->_possible = true;
  this->_value = 0;
  this->PAction = ActionPoint();
}

ActionClass::ActionClass(std::string name, ActionPoint PAction , int point){
  this->_priority = 0;
  this->_name = name;
  this->_done = false;
  this->_possible = true;
  this->_value = point;
  this->PAction = PAction;
}

// Setters
// void ActionClass::setName(std::string name){
//   this->_name = name;
// }
//
// void ActionClass::setPoint(int val){
//   this->_value = val;
// }
//
// void ActionClass::setDone(bool done){
//   this->_done = done;
// }

// Getters
// int ActionClass::getPoint(){
//   return this->_point;
// }
//
// bool ActionClass::getPossibility(){
//   return this->_possible;
// }

// Methodes

void ActionClass::updatePriority() {
  /* code */
}

void ActionClass::checkPossibility() {
  /* code */
}

void ActionClass::setSide(bool side) {
  if(side){
    PAction.startPoint.x = (1500 - PAction.startPoint.x) + 1500;
    PAction.endPoint.x = (1500 - PAction.endPoint.x) + 1500;
  }
}

std::ostream& operator<<(std::ostream& os, const ActionClass& AC)
{
    os << "ActionClass : { \n name: " << AC._name << "\n value: " << AC._value << "\n priority: " << AC._priority << "\n done: " << AC._done << "\n possible: " << AC._possible << "\n " << AC.PAction << std::endl;
    return os;
}


// 3000 largeur x
// 2000 longueur y

// cote vert standart
