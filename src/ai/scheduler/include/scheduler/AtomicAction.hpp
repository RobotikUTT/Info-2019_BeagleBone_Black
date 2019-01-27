/**  @file ActionClass.h
*    @brief represent a single action
*    
*    
*    @author Clément de La Bourdonnaye
*/
#ifndef ATOMIC_ACTION_H
#define ATOMIC_ACTION_H

#include "scheduler/ActionPoint.h"
#include "scheduler/Action.hpp"

#include <vector>
#include <string>

/**
 * @brief      Class for action class.
 */
class AtomicAction : public Action {
public:
  AtomicAction(std::string name, std::string performer);

private:
  std::string performer;
};

std::ostream& operator<<(std::ostream& os, const AtomicAction& AC);

#endif
