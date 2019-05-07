#include "action_manager/ActionPerformer.hpp"

#include "ai_msgs/ActionStatus.h"

class FailActionPerformer : public ActionPerformer {
	FailActionPerformer() : ActionPerformer("fail") {
		this->setNodeStatus(NodeStatus::READY);
	}

	void start() override {
		this->returns(ActionStatus::PAUSED);
	}

	ActionPoint computeActionPoint(Argumentable* args, Pose2D robotPos) override {
		ActionPoint res;
		res.start = robotPos;
		res.end = robotPos;
		return res;
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "fail");

	FailActionPerformer performer();
	ros::spin();

	return 0;
}
