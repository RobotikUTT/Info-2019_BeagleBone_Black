#include "action_manager/ActionPerformer.hpp"

// TODO take robot status into consideration to handle ROBOT_HALT

ActionPerformer::ActionPerformer(std::string name) : name(name), nh() {
	// Service for action point computation
	actionPointSrv  = nh.advertiseService(
		getActionPointService(name),
		&ActionPerformer::_computeActionPoint,
		this
	);

	// Suscribe to robot status
	robotWatcherSub = nh.subscribe("/ai/robot_watcher/robot_status", 1, &ActionPerformer::onRobotStatus, this);

	// Action server for this action
	actionServer = new PerformActionSrv(
		getActionServer(name),
		false
	);

	actionServer->registerGoalCallback(boost::bind(&ActionPerformer::onGoal, this));
	actionServer->registerPreemptCallback(boost::bind(&ActionPerformer::onPreempt, this));
	actionServer->start();
}

void ActionPerformer::ready() {
	service_ready("performer", name, 1);
}

bool ActionPerformer::_computeActionPoint(ai_msgs::ComputeActionPoint::Request& req, ai_msgs::ComputeActionPoint::Response& res) {
	// Extract data from request and call performer function
	ActionPoint result = computeActionPoint(
		&req.args,
		Point(req.robot_pos.x, req.robot_pos.y, req.robot_pos.rot)
	);

	// Copy result to response
	res.start_point.x = result.startPoint.x;
	res.start_point.y = result.startPoint.y;
	res.start_point.rot = result.startPoint.angle;
	res.end_point.x = result.endPoint.x;
	res.end_point.y = result.endPoint.y;
	res.end_point.rot = result.endPoint.angle;
	
	return true;
}

/**
 *  Goal received
 */
void ActionPerformer::onGoal() {
	ai_msgs::PerformGoal::ConstPtr goal = actionServer->acceptNewGoal();

	// save args
	_args = goal->arguments;

	// run action
	start();
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
	result.status = ACTION_DONE;

	// Send back to client
	actionServer->setSucceeded(result);
}

/**
 * Pause the action
 */
void ActionPerformer::actionPaused() {
	// Create result message
	ai_msgs::PerformResult result;
	result.status = ACTION_PAUSED;

	// Send back to client
	actionServer->setSucceeded(result);
}

/**
 *  Monitor robot status to catch HALT signal
 */
void ActionPerformer::onRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg) {
  if (msg->robot_status == ROBOT_HALT){
	actionServer->shutdown();
  }
}

/**
 *  Retrieve an argument from the argument list
 */
double ActionPerformer::getArg(std::string name, double defaultValue /*= 0*/, std::vector<ai_msgs::Argument>* args /*= NULL*/) {
	// No arg list provided -> use current action's one
	if (args == NULL) {
		args = &_args;
	}

	// Seek an argument with this name
	for (const auto& next : *args) {
		if (name.compare(next.name) == 0) {
			return next.value;
		}
	}

	return defaultValue;
}

bool ActionPerformer::hasArg(std::string name, std::vector<ai_msgs::Argument>* args /*= NULL*/) {
	// No arg list provided -> use current action's one
	if (args == NULL) {
		args = &_args;
	}

	// Seek an argument with this name
	for (const auto& next : *args) {
		if (name.compare(next.name) == 0) {
			return true;
		}
	}

	return false;
}