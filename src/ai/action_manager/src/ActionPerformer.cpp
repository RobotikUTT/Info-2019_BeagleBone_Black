#include "action_manager/ActionPerformer.hpp"


ActionPerformer::ActionPerformer(std::string name) : Node(name, "action"), name(name) {
	// Service for action point computation
	actionPointSrv  = nh.advertiseService(
		getActionPointService(name),
		&ActionPerformer::_computeActionPoint,
		this
	);

	// Suscribe to robot status
	// TODO remove to handle only cancelling via scheduler
	robotWatcherSub = nh.subscribe("/ai/controller/robot_status", 1, &ActionPerformer::onRobotStatus, this);

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
		_args = goal->arguments;
	
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
void ActionPerformer::actionPerformed() {
	// Create result message
	ai_msgs::PerformResult result;
	result.status.state_code = ActionStatus::DONE;

	// Send back to client
	actionServer->setSucceeded(result);
}

/**
 * Pause the action
 */
void ActionPerformer::actionPaused() {
	// Create result message
	ai_msgs::PerformResult result;
	result.status.state_code = ActionStatus::PAUSED;

	// Send back to client
	actionServer->setSucceeded(result);
}

/**
 *  Monitor robot status to catch HALT signal
 */
void ActionPerformer::onRobotStatus(const RobotStatus::ConstPtr& msg) {
  if (msg->robot_status == RobotStatus::ROBOT_HALT){
	actionServer->shutdown();
  }
}

