#include "action_manager/ActionPerformer.hpp"

#include "ai_msgs/ActionStatus.h"

class MoveActionPerformer : public ActionPerformer {
	MoveActionPerformer() : ActionPerformer("move") {
		this->setNodeStatus(NodeStatus::READY);
	}

	void start() override {
		this->actionPerformed()
	}
}
/**
 * @brief object constructor
 *
 * @param[in] name name of the action server
 */
IdleActionPerfomer::IdleActionPerfomer(std::string name) : ActionPerformer(name) {
	setNodeStatus(NodeStatus::READY);
}

ActionPoint IdleActionPerfomer::computeActionPoint(Argumentable* actionArgs, Pose2D robotPos) {
	ActionPoint point;
	point.start = robotPos;
	point.end = robotPos;
	return point;
}

/**
 * @brief run action toward a new goal and send the appropriate to the STM
 */
void IdleActionPerfomer::start() {
	ros::Duration(getLong("duration", 1)).sleep();
	this->actionPerformed();
}

void IdleActionPerfomer::cancel() {}

int main(int argc, char** argv) {
	ros::init(argc, argv, "idle");

	IdleActionPerfomer performer("idle");
	ros::spin();

	return 0;
}
