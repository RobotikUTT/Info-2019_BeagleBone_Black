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

/**
 * Compute the estimated distance to travel before the robot reach the end of the action
 */
int ActionBlock::distanceToTravel(Point& robot_pos) {
    int distance = 0;
    ActionPoint* currentActionPoint = new ActionPoint(robot_pos, robot_pos);
    ActionPoint* nextActionPoint = NULL;
    
    for (auto& next : _actions) {
        nextActionPoint = next.actionPoint(currentActionPoint->endPoint);

        // Add distance between the end of the previous action and the begin of this one
        distance += next.distanceToTravel(currentActionPoint->endPoint);
    }

    return  distance;
}

/**
 * Compute the initial and final point of the action
 */
ActionPoint* ActionBlock::actionPoint(Point& previousActionPoint) {
    // Test whether the action point was already computed
    if (_actionPoint != NULL) {
        return _actionPoint;
    }

    Point* start = &_actions.front().actionPoint(previousActionPoint)->startPoint;
    Point& current = previousActionPoint;
    ActionPoint* actionPoint;

    // Compute all actionPoints
    for (auto& next : _actions) {
        actionPoint = next.actionPoint(current);
        current = actionPoint->endPoint;

        if (start == NULL) {
            start = &actionPoint->startPoint;
        }
    }

    // take first and last position
    _actionPoint = new ActionPoint(
        *start, // first startPoint
        current // last endPoint
    );

    return _actionPoint;
}