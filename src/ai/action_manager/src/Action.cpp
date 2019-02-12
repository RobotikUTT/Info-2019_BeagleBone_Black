/** @file Action.cpp
*	@brief class for generic action
* 	@author Clément de La Bourdonnaye
*/
#include "action_manager/Action.hpp"

/**
 * @brief default constructor
 */
Action::Action(std::string name)
	: _name(name), _state(ActionStatus::IDLE), _points(0), _sync(false), _actionPoint(NULL) { }

/**
 * @brief return the priority of the action according to it's points and distance
 *
 * @param robot_pos current robot position
 */
double Action::priority(Point& robot_pos) {
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
double Action::distanceToTravel(Point& robot_pos) {
	return actionPoint(robot_pos).startPoint.manhattanDist(robot_pos) +
		actionPoint(robot_pos).distance();
}

// Virtual functions (have to be redefined into child's classes)
int Action::points() const { return _points; }


// Comparison operator
bool Action::equals(const Action& b) const {
	// First try basic tests
	return getBasePoints() == b.getBasePoints() &&
		isSync() == b.isSync() &&
		name() == b.name();
}

void Action::display(std::ostream& os) const {
	os << "[" << name()
				<< "] points=" << points()
				<< ", sync=" << isSync();
}


// Stream output from action
std::ostream& operator<<(std::ostream& os, const Action& ac) {
	ac.display(os);

	return os;
}

/*
 *	Static names factory
 */
std::string getActionPointService(std::string performer) {
	std::ostringstream output;
	output << "action_manager/" << performer << "/actionpoint";
	return output.str();
}
std::string getActionServer(std::string performer) {
	std::ostringstream output;
	output << "action_" << performer;
	return output.str();
}