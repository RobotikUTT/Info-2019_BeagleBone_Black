#include "action_manager/PerformClient.hpp"

PerformClient::PerformClient(std::string name, std::string package) : Node(name, package) { }

void PerformClient::performAction(AtomicAction& action, Point robot_pos) {
	// TODO check for previously launched clients
	client = new PerformActionClt(getActionServer(action.performer()), true);
	
	// wait for the action server to start
	client->waitForServer(); // TODO wait for finite time and throw

	// send a goal to the action
	ai_msgs::PerformGoal goal;
	goal.arguments = action.getArgs();
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
