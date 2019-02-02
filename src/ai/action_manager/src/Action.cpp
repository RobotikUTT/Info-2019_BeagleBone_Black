/** @file Action.cpp
*    @brief class for generic action
*    
*    
*    @author Alexis CARE
*    @author Clément de La Bourdonnaye
*/
#include "action_manager/Action.hpp"

/**
 * @brief      Constructs the default object.
 */
Action::Action(std::string name) :_name(name), _state(ACTION_IDLE), _points(0), _sync(false), _actionPoint(NULL) {

}

/**
 * @brief return the priority of the action according to it's points and distance
 *
 * @param robot_pos current robot position
 */
int Action::priority(Point& robot_pos) {
  int action_points = points();
  int distance = distanceToTravel(robot_pos);
  
  return action_points * action_points / distance;
}

// Getters
std::string Action::name() const {
  return _name;
}

int Action::getBasePoints() const {
  return _points;
}

int Action::state() const {
  return _state;
}

bool Action::isSync() const {
  return _sync;
}

// Setters
void Action::setSync(bool sync) {
  _sync = sync;
}

void Action::setBasePoints(int points) {
  _points = points;
}

/**
 * Compute the distance to travel before the robot reach the end
 */
int Action::distanceToTravel(Point& robot_pos) {
  return actionPoint(robot_pos)->startPoint.manhattanDist(robot_pos) +
    actionPoint(robot_pos)->distance();
}

// Virtual functions (have to be redefined into child's classes)
int Action::points() const { return 0; }
ActionPoint* Action::actionPoint(Point& previousActionPoint) {
  return _actionPoint;
}