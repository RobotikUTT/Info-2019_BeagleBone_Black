#include "action_manager/ActionBlock.hpp"

ActionBlock::ActionBlock(Action descriptor, std::list<Action> actions) :
    Action(descriptor.name()), _actions(actions) {
    // Copy descriptor useful properties
    setBasePoints(descriptor.points());
    setSync(descriptor.isSync());
}

/**
 * Compute the sum of points earned by each value
 */
int ActionBlock::points() {
    int points = _points;

    for (auto& next : _actions) {
        points += next.points();
    }
    
    return points;
}

Point ActionBlock::startPoint() {
    return Point();
}