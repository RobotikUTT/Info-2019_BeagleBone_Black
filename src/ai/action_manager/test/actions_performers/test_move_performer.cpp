#include "action_manager/ActionPerformer.hpp"

#include "ai_msgs/ActionStatus.h"

class MoveActionPerformer : public ActionPerformer {
	MoveActionPerformer() : ActionPerformer("move") {
		this->setNodeStatus(NodeStatus::READY);
	}

	void start() override {
		this->returns(ActionStatus::PAUSED);
	}

	ActionPoint computeActionPoint(Argumentable* args, Pose2D robotPos) override {
		ActionPoint res;

		res.start = robotPos;
		res.end.x = args->getDouble("x", 0);
		res.end.y = args->getDouble("y", 0);
		res.end.theta = args->getDouble("angle", 0);
		
		return res;
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "move");

	MoveActionPerformer performer();
	ros::spin();

	return 0;
}
