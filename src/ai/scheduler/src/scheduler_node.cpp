#include "scheduler/scheduler_node.hpp"

/**
 * @brief object constructor
 *
 * @param n NodeHandler
 */
Scheduler::Scheduler() : PerformClient() {
	// Retrieve action
	std::string actions_file;
	nh.getParam("scheduler/actions_directory", actions_file);

	this->side = SIDE_GREEN;
	this->side_sub = nh.subscribe("side", 1, &Scheduler::setSide, this);

	this->control_srv = nh.advertiseService("scheduler/do", &Scheduler::setState, this);
	

	// Parse actions file
	ActionsParser parser(ActionFilePath("main", actions_file));
	ActionPtr root = parser.getAction();

	service_ready("ai", "scheduler", 1);
}

bool Scheduler::setState(ai_msgs::SetSchedulerState::Request &req, ai_msgs::SetSchedulerState::Response &res) {
	// If there is a change in state
	if (this->running != req.running) {
		// apply change
		if (req.running) {
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

/**
 * @brief Sets the starting side.
 *
 * @param[in] msg The SetSide message
 */
void Scheduler::setSide(const ai_msgs::SetSide::ConstPtr& msg){
	if(this->side != msg->side) {
		this->side = ! this->side;
		// TODO acm
		// this->actionManager.changeSide();
	}
}


int main(int argc, char *argv[]) {
	ros::init(argc,argv, "scheduler_node");

	Scheduler node();
	ros::spin();
}
