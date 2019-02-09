#include "action_manager/ActionBlock.hpp"

ActionBlock::ActionBlock(std::string name) :
	Action(name), _actions() {}

/**
 * Compute the sum of points earned by each value
 */
int ActionBlock::points() const {
	int points = _points;

	for (auto& next : _actions) {
		points += next->points();
	}
	
	return points;
}

std::list<ActionPtr> ActionBlock::subactions() const {
	return _actions;
}

void ActionBlock::addAction(ActionPtr action) {
	// copy and store reference here
	_actions.push_back(std::move(action));
}

/**
 * Compute the estimated distance to travel before the robot reach the end of the action
 */
double ActionBlock::distanceToTravel(Point& robot_pos) {
	int distance = 0;
	Point currentPoint = robot_pos;
	
	for (auto& next : _actions) {
		// Add distance between the end of the previous action and the begin of this one
		distance += next->distanceToTravel(currentPoint);

		// Then compute new action point
		currentPoint = next->actionPoint(currentPoint).endPoint;
	}

	return distance;
}

/**
 * Compute the initial and final point of the action
 */
ActionPoint& ActionBlock::actionPoint(Point& previousPoint) {
	// Test whether the action point was already computed
	if (_actionPoint != NULL) {
		return *_actionPoint;
	}

	Point* start = NULL; // computed later
	Point& current = previousPoint;
	ActionPoint actionPoint;

	// Compute all actionPoints
	for (auto& next : _actions) {
		actionPoint = next->actionPoint(current);
		current = actionPoint.endPoint;

		if (start == NULL) {
			start = &actionPoint.startPoint;
		}
	}

	// take first and last position
	_actionPoint = std::make_shared<ActionPoint>(
		*start, // first startPoint
		current // last endPoint
	);

	return *_actionPoint;
}

// Helped with https://stackoverflow.com/questions/2825424/comparing-objects-and-inheritance
bool ActionBlock::equals(const Action& action) const  {
	const ActionBlock* aBlock = dynamic_cast<const ActionBlock*>(&action);

	// Try to cast for comparison and compare with action's default comparison
	if (aBlock != NULL && Action::equals(action)) {
		// Then test for subactions
		std::list<ActionPtr> largs = subactions();
		std::list<ActionPtr> rargs = aBlock->subactions();

		// as many args
		if (largs.size() != rargs.size()) return false;

		std::list<ActionPtr>::iterator rit = rargs.begin();
		std::list<ActionPtr>::iterator lit = largs.begin();

		while (rit != rargs.end()) {
			if (!(*rit)->equals(**lit)) {
				return false;
			}

			rit ++;
			lit ++;
		}

		// if all subactions have proven to be equal
		// we can ensure blocks are the same
		return true;
	}

	return false;
}

void ActionBlock::display(std::ostream& os) const {
	std::stringstream content;

	// Output not indented title
	Action::display(os);

	// Create content to be indented
	for (const auto& next : subactions()) {
		content << std::endl << *next;
	}

	std::string line;

	while(std::getline(content, line, '\n')) {
		os << '\t' << line << '\n';
	}
}