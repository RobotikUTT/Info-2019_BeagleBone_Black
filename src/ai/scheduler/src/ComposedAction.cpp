#include "scheduler/ComposedAction.hpp"

ComposedAction::ComposedAction(std::string name, std::list<Action> actions) : Action(name), _actions(actions) {

}



int ComposedAction::points() {
    // todo sum of list
    return 0;
}

Point ComposedAction::startPoint() {
    return Point();
}