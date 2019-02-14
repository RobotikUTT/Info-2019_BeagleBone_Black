#include "action_manager/PerformClient.hpp"

PerformClient::PerformClient(std::string name, std::string package) : Node(name, package) { }

void PerformClient::performAction(AtomicActionPtr action, Point robot_pos) {
	// TODO check for previously launched clients
	client = new PerformActionClt(getActionServer(action->performer()), true);
	
	// wait for the action server to start
	client->waitForServer(); // TODO wait for finite time and throw

	// send a goal to the action
	ai_msgs::PerformGoal goal;
	goal.arguments = action->getArgs();
	goal.robot_pos.x = robot_pos.x;
	goal.robot_pos.y = robot_pos.x;
	goal.robot_pos.rot = robot_pos.angle;

	client->sendGoal(
		goal,
		boost::bind(&PerformClient::onFinished, this, _1, _2),
		PerformActionClt::SimpleActiveCallback(),
		PerformActionClt::SimpleFeedbackCallback()
	);
}

void PerformClient::cancelAction() {
	client->cancelGoal();
	onPaused();
}

void PerformClient::getRequired(std::vector<NodeRequirement>& requirements, ActionPtr action) {
	// Try to cast as block to add all subactions requirements
	const auto block = std::dynamic_pointer_cast<ActionBlock>(action);
	if (block) {
		for (const auto& next : block->subactions()) {
			getRequired(requirements, next);
		}
	}

	// Try to cast as block to add all subactions requirements
	const auto atomic = std::dynamic_pointer_cast<AtomicAction>(action);
	if (atomic) {
		std::string perfomer = getActionNodePath(atomic->performer());

		// Check if the performer is not already required
		for (const auto& nextReq : requirements) {
			if (nextReq.nodename == perfomer) {
				return;
			}
		}

		// Add to the list otherwise
		NodeRequirement req;
		req.nodename = perfomer;
		req.optional = false;

		requirements.push_back(req);
	}
}