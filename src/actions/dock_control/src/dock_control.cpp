//!/usr/bin/python3

from action_manager import ActionPerformer
from args_lib.argumentable import Argumentable

from typing import Dict
from geometry_msgs.msg import Pose2D
from ai_msgs.msg import ActionPoint, NodeStatus, ActionStatus

from interface_msgs.msg import CanData

import rospy

class DockControlActionPerformer : public ActionPerformer {
private:
	Subscriber can_in;
	Published can_out;
public:
	DockControlActionPerformer() {
		super().__init__("dock_control")
		this->can_in = nh.subscribe("/can_interface/in", 1, &ReachActionPerformer::onCanData, this);
		this->can_out = nh.advertise<interface_msgs::CanData>("/can_interface/out", 1);

		this->setNodeStatus(NodeStatus.READY)	
	}

	void onCanData(const interface_msgs::CanData::ConstPtr& msg){
		if (msg->type == "your_dock_has_fullfilled_your_request") {
			// It's ok !
			this->returns(ActionStatus::DONE);
		}
	}
			

	ActionPoint computeActionPoint(Argumentable* actionArgs, Pose2D robotPos) {
		// Compute start and end of action
		ActionPoint result;
		result.start = robotPos;
		result.end = robotPos;
		return result;
	}
	
	void start() override {
		msg = CanData()
		msg.type = "set_dock_height"
		msg.params = this->toList()

		this->can_out.publish(msg)


int main(int argc, char** argv) {
	ros::init(argc, argv, "dock_control");

	DockControlActionPerformer performer("dock_control");
	ros::spin();

	return 0;
}