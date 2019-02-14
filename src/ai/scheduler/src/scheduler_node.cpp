#include "scheduler/scheduler_node.hpp"

/**
 * @brief object constructor
 *
 * @param n NodeHandler
 */
Scheduler::Scheduler() : PerformClient("scheduler", "ai"), side(Side::LEFT), robotPosition() {
	// Retrieve action
	std::string actions_file;
	nh.getParam("actions_directory", actions_file);

	this->side = Side::LEFT;
	this->control_srv = nh.advertiseService("/scheduler/do", &Scheduler::setState, this);

	this->robotPosition_sub = nh.subscribe<can_msgs::Point>("/STM/Position", 10, &Scheduler::setRobotPosition, this);

	// Parse actions file
	try {
		ActionsParser parser(ActionFilePath("main", actions_file));
		this->rootAction = parser.getAction();
	} catch(const char* error) {
		ROS_ERROR_STREAM("Unable to initialize scheduler due to parsing error: " << error);
		setNodeStatus(NodeStatus::NODE_ERROR, 1);
		return;
	}

	std::vector<NodeRequirement> reqs;
	this->getRequired(reqs, this->rootAction);

	this->waitForNodes(reqs, 5);
}

void Scheduler::onWaitingResult(bool success) {
	if (success) {
		setNodeStatus(NodeStatus::NODE_READY);
	} else {
		ROS_ERROR_STREAM("Some actions are missing, unable to start node");
		setNodeStatus(NodeStatus::NODE_ERROR, 2);
		return;
	}
}

bool Scheduler::setState(SetSchedulerState::Request &req, SetSchedulerState::Response &res) {
	// If there is a change in state
	if (this->running != req.running) {
		this->running = req.running;
		
		// apply change
		if (req.running) {
			ROS_INFO("Scheduler actions resumed !");
			this->side = req.side;

			// Resume action
			this->nextAction();
		} else {
			ROS_INFO("Scheduler actions paused...");
			cancelAction();
		}
	}

	return true;
}

void Scheduler::nextAction() {
	if (!this->running) {
		return;
	}

	// If pause cancer has spread into the root of the action tree
	if (this->rootAction->state() == ActionStatus::PAUSED) {
		// We cure it
		this->rootAction->setState(ActionStatus::IDLE);
	}
	
	if (this->rootAction->state() == ActionStatus::IDLE) {
		ActionChoice choice =
			getOptimalNextAtomic(this->rootAction, this->robotPosition);

		ROS_INFO_STREAM("[Current action tree]" << std::endl << *this->rootAction);

		if (choice.action != NULL) {
			// save current action
			this->currentAction = choice.action;

			ROS_INFO_STREAM("Perfoming : " << *this->currentAction);

			// call then onfinished or onpause
			this->performAction(choice.action, this->robotPosition);
		} else {
			ROS_INFO_STREAM("No action to be performed");
			this->rootAction->setState(ActionStatus::DONE);
		}
	} else {
		ROS_INFO_STREAM("Root action is in state " << this->rootAction->state());
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

void Scheduler::setRobotPosition(const can_msgs::Point::ConstPtr& msg) {
	this->robotPosition.x = msg->pos_x;
	this->robotPosition.y = msg->pos_y;
	this->robotPosition.angle = msg->angle;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "scheduler_node");
	Scheduler node;
	ros::spin();
}
