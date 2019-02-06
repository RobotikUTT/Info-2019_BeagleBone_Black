#include "action_manager/PerformClient.hpp"

PerformClient::PerformClient() : nh() { }

void PerformClient::performAction(AtomicAction& action, procedures_msgs::OrPoint& robot_pos) {
	// TODO check for previously launched clients
	client = new PerformActionClt(getActionServer(action.performer()), true);

	// wait for the action server to start
	client->waitForServer(); // TODO wait for finite time and throw

	// send a goal to the action
	ai_msgs::PerformGoal goal;
	goal.arguments = action.getArgs();

	client->sendGoal(
		goal,
		boost::bind(&PerformClient::onFinished, this, _1, _2),
		PerformActionClt::SimpleActiveCallback(),
		PerformActionClt::SimpleFeedbackCallback()
	);
}
