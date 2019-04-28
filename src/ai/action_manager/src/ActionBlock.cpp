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
	// Set ourself as this little node's parent
	action->setParent(shared_from_this());

	// copy and store reference here
	_actions.push_back(std::move(action));
}

/**
 * Compute the estimated distance to travel before the robot reach the end of the action
 */
double ActionBlock::distanceToTravel(Pose2D& robot_pos) {
	int distance = 0;
	Pose2D currentPoint = robot_pos;
	
	for (auto& next : _actions) {
		// Add distance between the end of the previous action and the begin of this one
		distance += next->distanceToTravel(currentPoint);

		// Then compute new action point
		currentPoint = next->actionPoint(currentPoint).end;
	}

	return distance;
}

/**
 * Compute the initial and final point of the action
 */
ActionPoint& ActionBlock::actionPoint(Pose2D& previousPoint) {
	// Test whether the action point was already computed
	if (_actionPoint != NULL) {
		return *_actionPoint;
	}

	Pose2D* start = NULL; // computed later
	Pose2D& current = previousPoint;
	ActionPoint actionPoint;

	// Compute all actionPoints
	for (auto& next : _actions) {
		actionPoint = next->actionPoint(current);
		current = actionPoint.end;

		if (start == NULL) {
			start = &actionPoint.start;
		}
	}

	// take first and last position
	_actionPoint = std::make_shared<ActionPoint>();
	_actionPoint->start = *start; // first startPoint
	_actionPoint->end = current; // last endPoint
	

	return *_actionPoint;
}

/**
 * Set action block state, in case the action is requested to be paused
 * or finished, it first check all possible action inside the block is
 * paused or finished, and then pause parent, or do not pause
 */
void ActionBlock::setState(int state) {
	// If we set action as something else than waiting
	// we do a ascending recursion
	if (state != ActionStatus::IDLE) {
		// If we encounter a pause, the new state become paused
		bool translateToPause = false;

		// Check all children make the state valid
		for (const auto& next : subactions()) {
			// Action to be performed found
			if (next->state() == ActionStatus::IDLE) {
				// deny state
				return;
			} else if (next->state() == ActionStatus::PAUSED) {
				// If one action is done, it cannot be finished or impossible
				translateToPause = true;
			}

			// If it's a unfinished sync action
			if (next->isSync() && next->state() != ActionStatus::DONE) {
				// No need to proceed further, we can apply state
				break;
			}
		}

		// Apply state
		Action::setState(translateToPause ? ActionStatus::PAUSED : state);
	}
	
	// Otherwise if we resume action
	else if (this->state() == ActionStatus::PAUSED) {
		// We perform a descending unpause without questions
		for (const auto& next : subactions()) {
			if (next->state() == ActionStatus::PAUSED) {
				next->setState(ActionStatus::IDLE);
			}
		}
	}
}

// Helped with https://stackoverflow.com/questions/2825424/comparing-objects-and-inheritance
/*bool ActionBlock::equals(const Action& action) const  {
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
}*/

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