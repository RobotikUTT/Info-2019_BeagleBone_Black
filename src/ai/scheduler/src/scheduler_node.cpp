#include "scheduler/scheduler_node.hpp"

/**
 * @brief object constructor
 *
 * @param n NodeHandler
 */
Scheduler::Scheduler() : PerformClient("scheduler", "ai"), side(Side::LEFT) {
	// Retrieve action
	std::string actions_file;
	nh.getParam("scheduler/actions_directory", actions_file);

	this->side = Side::LEFT;
	this->control_srv = nh.advertiseService("/scheduler/do", &Scheduler::setState, this);

	// Parse actions file
	try {
		ActionsParser parser(ActionFilePath("main", actions_file));
		ActionPtr root = parser.getAction();
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
	// TODO run next action
	/*
	ActionChoice action = getOptimalNextAtomic(...)

	if (action.action != NULL) {
		// call then onfinished or onpause
		PerformClient::performAction(...);
	}
	*/
}

void Scheduler::onFinished(const actionlib::SimpleClientGoalState& state, const ai_msgs::PerformResultConstPtr& result) {
	// TODO mark current action as finished

	nextAction();
}

void Scheduler::onPaused() {
	// TODO mark current action as paused

	nextAction();
}

void Scheduler::stop() {
	// TODO stop current action (mark it as idle instead of paused to resume afterward)
}

void Scheduler::resume() {
	// TODO resume current action or run next one
}



int main(int argc, char *argv[]) {
	ros::init(argc, argv, "scheduler_node");
	
	Scheduler node;
	ros::spin();
}
