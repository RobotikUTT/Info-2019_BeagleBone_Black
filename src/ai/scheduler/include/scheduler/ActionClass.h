/**  @file ActionClass.h
*    @brief Class for ActionClass
*    
*    
*    @author Alexis CARE
*/
#ifndef ACTION_CLASS_H
#define ACTION_CLASS_H

#include "scheduler/ActionPoint.h"

#include <vector>
#include <string>

/**
 * @brief      Class for action class.
 */
class ActionClass{
public:

  int _priority;
  std::string _name;
  int8_t _action;
  bool _done;
  bool _possible;
  int _value;
  float _diff_factor;
  ActionPoint PAction;
  std::vector<int> _param;

  // make a constructor from json object ?
  ActionClass();
  ActionClass(std::string name, int8_t action, ActionPoint PAction , int point, float fact, std::vector<int> para);

  void updatePriority(Point& robot_pos);
  void checkPossibility();
  void changeSide();

  bool operator< ( const ActionClass &B) const;

private:
  // bonus point relation
};

std::ostream& operator<<(std::ostream& os, const ActionClass& AC);



#endif
