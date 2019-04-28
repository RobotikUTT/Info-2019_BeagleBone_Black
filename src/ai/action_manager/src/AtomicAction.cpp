#include "action_manager/AtomicAction.hpp"

AtomicAction::AtomicAction(std::string name, std::string performer) :
	Action(name), _performer(performer) { }


// Getters
std::string AtomicAction::performer() const {
	return _performer;
}

/**
 *	Return the action point according either to the previous action's actionPoint
 *	or to current coordinates.
 *
 * 	Calls associated performer service to let it compute the point based on args 
 */
ActionPoint& AtomicAction::actionPoint(Pose2D& previousPoint) {
	if (_actionPoint == NULL) {
		// create client
		ros::NodeHandle nh;
		ros::ServiceClient client = nh.serviceClient<ai_msgs::ComputeActionPoint>(
			getActionPointService(performer())
		);

		ai_msgs::ComputeActionPoint srv;
		srv.request.robot_pos = previousPoint;
		srv.request.args = this->toList();
		
		if (client.call(srv)) {
			_actionPoint = std::make_shared<ActionPoint>(srv.response.action_point);
		} else {
			throw "failed to call service" + getActionPointService(performer());
		}
	}
	
	return *_actionPoint;
}

// Equality
/*bool AtomicAction::equals(const Action& action) const {
	const AtomicAction* atomic = dynamic_cast<const AtomicAction*>(&action);

	// Cast succeed and basic properties also
	if (atomic != NULL && Action::equals(action) && performer() == atomic->performer()) {
		// Then test for arguments
		std::vector<ai_msgs::Argument> args = getArgs();
		std::vector<ai_msgs::Argument> compArgs = atomic->getArgs();

		// as many args
		if (args.size() != compArgs.size()) return false;

		for(const auto& lnext : args) {
			for(const auto& rnext : compArgs) {
				if (lnext.name == rnext.name) {
					if (lnext.value.type != rnext.value.type) {
						return false;
					}

					// If given value does not match
					if ((lnext.value.type == Value::FLOAT && lnext.value.floatValue != rnext.value.floatValue) ||
						(lnext.value.type == Value::DOUBLE && lnext.value.doubleValue != rnext.value.doubleValue) ||
						(lnext.value.type == Value::STRING && lnext.value.stringValue != rnext.value.stringValue)) {
						return false;
					}
					
				}
			}
		}

		return true;
	}

	return false;
}*/

void AtomicAction::display(std::ostream& os) const {
	Action::display(os);
	
	// Output additional infos to not indented title
	os << ", performer=" << performer();

	// Create content to be indented
	for (const auto& next : this->toList()) {
		os <<  std::endl << "\t- " << next.name << "=" << next.value;
	}
}