#include "scheduler/scheduler_node.hpp"

/**
 * @brief object constructor
 *
 * @param n NodeHandler
 */
Scheduler::Scheduler() : PerformClient("scheduler", "ai"), side(Side::LEFT), robotPosition() {
	// Retrieve action
	std::string actions_file;
	nh.getParam("scheduler/actions_directory", actions_file);

	this->side = Side::LEFT;
	this->control_srv = nh.advertiseService("/scheduler/do", &Scheduler::setState, this);

	this->robotPosition_sub = nh.subscribe<can_msgs::Point>("/STM/Position", 10, &Scheduler::setRobotPosition, this);

	// Parse actions file
	try {
		ActionsParser parser(ActionFilePath("main", actions_file));
		this->rootAction = parser.getAction();
	} catch(const char* error) {
		ROS_ERROR_STREAM("unable to initialize scheduler due to parsing error: " << error);
		setNodeStatus(NodeStatus::NODE_ERROR, 1);
		return;
	}
	
	setNodeStatus(NodeStatus::NODE_READY);
}

bool Scheduler::setState(SetSchedulerState::Request &req, SetSchedulerState::Response &res) {
	// If there is a change in state
	if (this->running != req.running) {
		// apply change
		if (req.running) {
			this->side = req.side;
			this->resume();
		} else {
			this->stop();
		}
	}

	this->running = req.running;

	return true;
}

void Scheduler::nextAction() {
	// If pause cancer has spread into the root of the action tree
	if (this->rootAction->state() == ActionStatus::PAUSED) {
		// We cure it
		this->rootAction->setState(ActionStatus::IDLE);
	}

	ActionChoice choice =
		ActionTools::getOptimalNextAtomic(this->rootAction, this->robotPosition);

	if (choice.action != NULL) {
		// save current action
		this->currentAction = choice.action;

		// call then onfinished or onpause
		this->performAction(choice.action, this->robotPosition);
	}
}

void Scheduler::onFinished(const actionlib::SimpleClientGoalState& state, const ai_msgs::PerformResultConstPtr& result) {
	currentAction->setState(ActionStatus::DONE);
	nextAction();
}

void Scheduler::onPaused() {
	currentAction->setState(ActionStatus::PAUSED);
	nextAction();
}

void Scheduler::stop() {
	// TODO stop current action (mark it as idle instead of paused to resume afterward)
}

void Scheduler::resume() {
	// TODO resume current action or run next one
}

void Scheduler::setRobotPosition(const can_msgs::Point::ConstPtr& msg) {
	this->robotPosition.x = msg.x;
	this->robotPosition.y = msg.y;
	this->robotPosition.angle = msg.rotation;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "scheduler_node");
	
	Scheduler node;
	ros::spin();
}
