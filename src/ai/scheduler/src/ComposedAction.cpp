#include "scheduler/ComposedAction.hpp"

ComposedAction::ComposedAction(std::string name, std::list<Action> actions) : Action(name), _actions(actions) {

}