#include "scheduler/AtomicAction.hpp"

AtomicAction::AtomicAction(std::string name, std::string performer) :
  Action(name), _performer(performer) { }


// Getters
int AtomicAction::points() {
  return _points;
}

Point AtomicAction::startPoint() {
    return Point();
}


list<Argument> AtomicAction::getArgs() {
  return _args;
}

// Setters
void AtomicAction::addArg(Argument arg) {
  _args.push_back(arg);
}