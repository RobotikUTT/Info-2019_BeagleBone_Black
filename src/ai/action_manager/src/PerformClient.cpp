#include "action_manager/PerformClient.hpp"

PerformClient::PerformClient(std::string name, std::string package) : Node(name, package) { }

void PerformClient::performAction(AtomicActionPtr action, OrientedPoint robot_pos) {
	// TODO check for previously launched clients
	client = new PerformActionClt(getActionServer(action->performer()), true);

	// wait for the action server to start
	client->waitForServer(); // TODO wait for finite time and throw

	// send a goal to the action
	ai_msgs::PerformGoal goal;
	goal.arguments = action->toList();
	goal.robot_pos = robot_pos;

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

void PerformClient::saveRequired(ActionPtr action) {
	// Try to cast as block to add all subactions requirements
	const auto block = std::dynamic_pointer_cast<ActionBlock>(action);
	if (block) {
		for (const auto& next : block->subactions()) {
			saveRequired(next);
		}
	}

	// Try to cast as block to add all subactions requirements
	const auto atomic = std::dynamic_pointer_cast<AtomicAction>(action);
	if (atomic) {
		std::string performer = getActionNodePath(atomic->performer());

		// Check if the performer is not already required
		if (this->isRequired(performer)) {
			return;
		}

		// Add to the list otherwise
		this->require(performer, false);
	}
}