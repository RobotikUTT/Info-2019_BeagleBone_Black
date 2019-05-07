#include "action_manager/ActionPerformer.hpp"

#include "ai_msgs/ActionStatus.h"

class SleepActionPerformer : public ActionPerformer {
	SleepActionPerformer() : ActionPerformer("sleep") {
		this->setNodeStatus(NodeStatus::READY);
	}

	void start() override {
		ros::Duration(this->getDouble("duration")).sleep();

		this->returns(ActionStatus::DONE);
	}

	ActionPoint computeActionPoint(Argumentable* args, Pose2D robotPos) override {
		ActionPoint res;
		res.start = robotPos;
		res.end = robotPos;
		return res;
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "sleep");

	SleepActionPerformer performer();
	ros::spin();

	return 0;
}
