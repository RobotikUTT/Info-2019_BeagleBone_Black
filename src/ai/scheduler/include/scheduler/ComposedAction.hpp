/**  @file ActionClass.h
*    @brief represent a single action
*    
*    
*    @author Clément de La Bourdonnaye
*/
#ifndef COMPOSED_ACTION_H
#define COMPOSED_ACTION_H

#include "scheduler/ActionPoint.h"
#include "scheduler/Action.hpp"

#include <vector>
#include <string>

/**
 * @brief      Class for action class.
 */
class ComposedAction : public Action {
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
  ComposedAction();
  ComposedAction(std::string name, int8_t action, ActionPoint PAction , int point, float fact, std::vector<int> para);

  void updatePriority(Point& robot_pos);
  void checkPossibility();
  void changeSide();

  bool operator< ( const ComposedAction &B) const;

private:
  // bonus point relation
};

std::ostream& operator<<(std::ostream& os, const ComposedAction& AC);

#endif
