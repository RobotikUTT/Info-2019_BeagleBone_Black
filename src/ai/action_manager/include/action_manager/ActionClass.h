#ifndef ACTION_CLASS_H
#define ACTION_CLASS_H

#include <string>
#include "ActionPoint.h"

class ActionClass{
public:

  int _priority;
  std::string _name;
  bool _done;
  bool _possible;
  int _value;
  ActionPoint PAction;


  ActionClass();
  ActionClass(std::string, ActionPoint, int);

  // void setName(std::string);
  // void setPoint(int);
  // void setDone(bool);

  // int getPoint();
  // bool getPossibility();

  void updatePriority();
  void checkPossibility();
  void setSide(bool);

  bool operator< ( const ActionClass &B) const {
    return _priority < B._priority;
  }

private:
  // bonus point relation
  // procedure
};

std::ostream& operator<<(std::ostream&, const ActionClass&);



#endif
