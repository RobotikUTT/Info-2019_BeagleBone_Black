#include "scheduler/AtomicAction.hpp"

AtomicAction::AtomicAction(std::string name, std::string performer) :
  Action(name), _performer(performer) {
  
}

int AtomicAction::points() {
  return 0;
}

Point AtomicAction::startPoint() {
    return Point();
}