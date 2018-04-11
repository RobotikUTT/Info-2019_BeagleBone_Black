#ifndef ACTION_CLASS_H
#define ACTION_CLASS_H

#include <string>
#include "action_manager/ActionPoint.h"

class ActionClass{
public:

  int _priority;
  std::string _name;
  bool _done;
  bool _possible;
  int _value;
  float _diff_factor;
  ActionPoint PAction;


  ActionClass();
  ActionClass(std::string, ActionPoint, int, float);

  void updatePriority(Point&);
  void checkPossibility();
  void changeSide();

  bool operator< ( const ActionClass &B) const;

private:
  // bonus point relation
  // procedure
};

std::ostream& operator<<(std::ostream&, const ActionClass&);



#endif
