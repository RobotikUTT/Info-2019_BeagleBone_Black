#ifndef ATOMIC_ACTION_H
#define ATOMIC_ACTION_H

#include "scheduler/ActionPoint.h"
#include "scheduler/Action.hpp"

#include <string>

/**
 * @brief      Class for action class.
 */
class AtomicAction : public Action {
public:
  AtomicAction(std::string name, std::string performer);

  std::string performer();

private:
  std::string _performer;
};

// TODO (or use a library?)
//std::ostream& operator<<(std::ostream& os, const AtomicAction& obj);

#endif
