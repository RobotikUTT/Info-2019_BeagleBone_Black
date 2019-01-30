/** @file ActionClass.cpp
*    @brief Class for ActionClass.
*    
*    
*    @author Alexis CARE
*/
#include "action_manager/Action.hpp"

/**
 * @brief      Constructs the default object.
 */
Action::Action(std::string name) :_name(name), _state(ACTION_IDLE), _points(0), _sync(false) {

}

/**
 * @brief      Return the priority of the action according to it's points and distance
 *
 * @param      robot_pos  current robot position
 */
int Action::priority(Point& robot_pos) {
  int action_points = points();
  int distance = distanceToTravel(robot_pos);
  //actionPoint().startPoint.manhattanDist(robot_pos)
  return action_points * action_points / distance;
}

// Getters
std::string Action::name() {
  return _name;
}

int Action::state() {
  return _state;
}

bool Action::isSync() {
  return _sync;
}

void Action::setSync(bool sync) {
  _sync = sync;
}

// Points setter
void Action::setBasePoints(int points) {
  _points = points;
}

/*void ActionClass::changeSide(){
  PAction.startPoint.y = (1500 - PAction.startPoint.y) + 1500;
  PAction.startPoint.angle *= -1;
  if (PAction.endPoint.y >= 0 ) {
    PAction.endPoint.y = (1500 - PAction.endPoint.y) + 1500;
    PAction.endPoint.angle *= -1;
  }
}*/

/**
 * @brief      Operator '<' for two ActionClass
 *
 * @param[in]  B     Other ActionClass
 *
 * @return     This < Other
 */
/*bool Action::operator< ( const ActionClass &B)const{
  if (_done) return 0;
  return (_priority < B._priority);
}*/

int Action::points() {
    // todo sum of list
    return 0;
}

/**
 * Compute the distance to travel before the robot reach the end
 */
int Action::distanceToTravel(Point& robot_pos) {
  return actionPoint().startPoint.manhattanDist(robot_pos) +
    actionPoint().distance();
}

ActionPoint Action::actionPoint() {
    return ActionPoint();
}