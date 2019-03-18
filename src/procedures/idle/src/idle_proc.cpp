#include "idle/idle_proc.hpp"


/**
 * @brief object constructor
 *
 * @param[in] name name of the action server
 */
IdleActionPerfomer::IdleActionPerfomer(std::string name) : ActionPerformer(name) {
	setNodeStatus(NodeStatus::READY);
}

ActionPoint IdleActionPerfomer::computeActionPoint(std::vector<ai_msgs::Argument>* actionArgs, OrientedPoint robotPos) {
	ActionPoint point;
	point.start = robotPos;
	point.end = robotPos;
	return point;
}

/**
 * @brief run action toward a new goal and send the appropriate to the STM
 */
void IdleActionPerfomer::start() {
	ros::Duration(getArg("duration", 1)).sleep();
	this->actionPerformed();
}

void IdleActionPerfomer::cancel() {}

int main(int argc, char** argv) {
	ros::init(argc, argv, "idle");

	IdleActionPerfomer performer("idle");
	ros::spin();

	return 0;
}
