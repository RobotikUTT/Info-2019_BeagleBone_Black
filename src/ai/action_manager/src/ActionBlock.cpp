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
int ActionBlock::points() const {
    int points = _points;

    for (auto& next : _actions) {
        points += next.points();
    }
    
    return points;
}

std::list<Action> ActionBlock::subactions() const {
    return _actions;
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

bool operator==(const ActionBlock& lhs, const ActionBlock& rhs){
    // First try basic tests
    bool baseTests = lhs.isSync() == rhs.isSync() && // sync
    lhs.name() == rhs.name(); // name

    if (!baseTests) return false;

    // Then test for subactions
    std::list<Action> largs = lhs.subactions();
    std::list<Action> rargs = rhs.subactions();

    // as many args
    if (largs.size() != rargs.size()) return false;

    std::list<Action>::iterator rit = rargs.begin();
    std::list<Action>::iterator lit = largs.begin();

    ActionBlock* br;
    ActionBlock* bl;
    AtomicAction* ar;
    AtomicAction* al;

    Action* rpoint;
    Action* lpoint;

    while (rit != rargs.end()) {
        rpoint = &(*rit);
        lpoint = &(*lit);

        br = dynamic_cast<ActionBlock*> (rpoint);
        bl = dynamic_cast<ActionBlock*> (lpoint);

        // Both action block
        if (br != 0 && bl != 0) {
            // TODO test if equality is well used
            if (br != bl) {
                return false;
            }
        } else {
            ar = dynamic_cast<AtomicAction*> (rpoint);
            al = dynamic_cast<AtomicAction*> (lpoint);

            // or both atomic action
            if (ar == 0 || al == 0 || ar != al) {
                return false;
            }
        }

        rit ++;
        lit ++;
    }

    return true;
}