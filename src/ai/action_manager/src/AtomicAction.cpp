#include "action_manager/AtomicAction.hpp"

AtomicAction::AtomicAction(std::string name, std::string performer) :
	Action(name), _performer(performer), _actionPoint(NULL) { }


// Getters
std::string AtomicAction::performer() const {
	return _performer;
}

/**
 *	Return the action point according either to the previous action's actionPoint
 *	or to current coordinates
 */
ActionPoint* AtomicAction::actionPoint(Point& previousActionPoint) {
	if (_actionPoint == NULL) {
		ActionPoint point;

		// TODO compute action point with perfomer help

		if (/*performer.isActionPointStatic()*/true) {
			_actionPoint = &point;
			return _actionPoint;
		}
	} else {
		return _actionPoint;
	}
}


std::list<ai_msgs::Argument> AtomicAction::getArgs() const {
	return _args;
}

// Setters
void AtomicAction::addArg(ai_msgs::Argument arg) {
	_args.push_back(arg);
}

// Equality
bool AtomicAction::equals(const Action& action) {
	const AtomicAction* atomic = dynamic_cast<const AtomicAction*>(&action);

	// Cast succeed and basic properties also
	if (atomic != NULL && Action::equals(action) && performer() == atomic->performer()) {
		// Then test for arguments
		std::list<ai_msgs::Argument> args = getArgs();
		std::list<ai_msgs::Argument> compArgs = atomic->getArgs();

		// as many args
		if (args.size() != compArgs.size()) return false;

		for(const auto& lnext : args) {
			for(const auto& rnext : compArgs) {
				if (lnext.name == rnext.name && lnext.value != rnext.value) {
					return false;
				}
			}
		}

		return false;
	}

	return false;
}

std::ostream& operator<<(std::ostream& os, const AtomicAction& ac) {
	return os << "[" << ac.name()
		<< "] points=" << ac.points()
		<< ", sync=" << ac.isSync()
		<< ", performer=" << ac.performer();
}