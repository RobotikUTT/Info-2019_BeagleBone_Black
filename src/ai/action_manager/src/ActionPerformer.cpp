#include "action_manager/ActionPerformer.hpp"

/*
 *	Static names factory
 */
std::string getActionPointService(std::string performer) {
	std::ostringstream output;
	output << "/action/" << performer << "/actionpoint";
	return output.str();
}

std::string getActionServer(std::string performer) {
	std::ostringstream output;
	output << "/action/" << performer;
	return output.str();
}

std::string getActionNodePath(std::string performer) {
	std::ostringstream output;
	output << "/action/" << performer;
	return output.str();
}



ActionPerformer::ActionPerformer(std::string name) : Node(name, "action"), name(name) {
	// Service for action point computation
	actionPointSrv  = nh.advertiseService(
		getActionPointService(name),
		&ActionPerformer::_computeActionPoint,
		this
	);

	// Action server for this action
	actionServer = new PerformActionSrv(
		nh,
		getActionServer(name),
		false
	);

	actionServer->registerGoalCallback(boost::bind(&ActionPerformer::onGoal, this));
	actionServer->registerPreemptCallback(boost::bind(&ActionPerformer::onPreempt, this));
	actionServer->start();
}

bool ActionPerformer::_computeActionPoint(ai_msgs::ComputeActionPoint::Request& req, ai_msgs::ComputeActionPoint::Response& res) {
	Argumentable args;
	args.fromList(req.args);

	// Extract data from request and call performer function
	res.action_point = computeActionPoint(
		&args,
		req.robot_pos
	);
	
	return true;
}

/**
 *  Goal received
 */
void ActionPerformer::onGoal() {
	if (actionServer->isNewGoalAvailable()) {
		ai_msgs::PerformGoal::ConstPtr goal = actionServer->acceptNewGoal();

		// save args
		this->fromList(goal->arguments, true);
	
		// run action
		start();
	}
}

/**
 *  Preempt received
 */
void ActionPerformer::onPreempt() {
	// Cancel current goal if needed
	cancel();
	
	// Mark as preempted
	actionServer->setPreempted();
}

/**
 * Terminate the action
 */
void ActionPerformer::returns(int status) {
	// Create result message
	ai_msgs::PerformResult result;
	result.state = status;

	// Send back to client
	actionServer->setSucceeded(result);
}