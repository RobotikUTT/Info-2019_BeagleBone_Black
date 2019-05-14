#include "reach/reach_proc.hpp"


/**
 * @brief object constructor
 *
 * @param[in] name name of the action server
 */
ReachActionPerfomer::ReachActionPerfomer(std::string name) : ActionPerformer(name) {
	this->can_data_sub = nh.subscribe("/can_interface/in", 1, &ReachActionPerfomer::onCanData, this);
	this->can_data_pub = nh.advertise<interface_msgs::CanData>("/can_interface/out", 1);

	this->pathfinder_srv = nh.serviceClient<pathfinder::FindPath>("/ai/pathfinder/findpath");

	setNodeStatus(NodeStatus::READY);
}

ActionPoint ReachActionPerfomer::computeActionPoint(Argumentable* actionArgs, Pose2D robotPos) {
	ActionPoint result;
	result.start = robotPos;
	result.end.x = getLong("x", robotPos.x);
	result.end.y = getLong("y", robotPos.y);
	result.end.theta = getLong("angle", robotPos.theta);
	return result;
}

/**
 * @brief Callback called when the STM finished all move order
 * @param[in]  msg   The finish message
 */
void ReachActionPerfomer::onCanData(const interface_msgs::CanData::ConstPtr& msg){
	if (msg->type == "order_complete") {
		timerTimeout.stop();
		returns(ActionStatus::DONE);
	}
}

/**
 * @brief run action toward a new goal and send the appropriate to the STM
 */
void ReachActionPerfomer::start() {
	// Test that some actions are to be performed
	if (hasLong("x") * hasLong("y") + 2 * hasLong("angle") == 0) {
		ROS_ERROR_STREAM("missing data in message, need at least (x,y) or (angle)");
		returns(ActionStatus::ERROR);
		return;
	}
	
	// If movement, get pathfinding data
	if (hasLong("x") && hasLong("y")) {
		geometry_msgs::Pose2D posEnd;
		posEnd.x = getLong("x");
		posEnd.y = getLong("y");

		pathfinder::FindPath srv;
		srv.request.posStart = this->robotPos;
		srv.request.posEnd = posEnd;

		if (this->pathfinder_srv.call(srv)) {
			// If no path found
			if (srv.response.return_code != pathfinder::FindPath::Response::PATH_FOUND) {
				ROS_WARN_STREAM("No path found to reach action goal...");
				this->returns(ActionStatus::PAUSED);
				return;
			}

			// Else give order to move along path
			for (auto& pose : srv.response.path) {
				this->moveTo(pose);
			}
		} else {
			ROS_ERROR_STREAM("Error while calling pathfinder service");
			this->returns(ActionStatus::ERROR);
		}
	}

	// If an angle is provided
	if (hasLong("angle")) {
		interface_msgs::CanData msg;
		msg.type = "rotate";
		
		Argumentable param;
		param.setLong("angle", getLong("angle"));
		msg.params = param.toList();
		this->can_data_pub.publish(msg);
	}

	int timeout = getLong("timeout", 0);
	if (timeout > 0) {
		timerTimeout = nh.createTimer(ros::Duration(timeout), &ReachActionPerfomer::timeoutCallback , this, true);
	}
}

void ReachActionPerformer::moveTo(geometry_msgs::Pose2D location) {
	Argumentable params;
	params.setLong("x", location.x);
	params.setLong("y", location.y);
	params.setLong("angle", location.theta);
	params.setLong("direction", interface_msgs::Directions::FORWARD);

	interface_msgs::CanData msg;
	msg.type = "go_to";
	msg.params = params.toList();

	this->can_data_pub.publish(msg);
}

void ReachActionPerfomer::cancel() {
	// reset all goal in the STM
	Argumentable params;
	interface_msgs::CanData msg;
	msg.type = "set_stm_mode";

	params.setLong("mode", interface_msgs::StmMode::RESET_ORDERS);
	msg.params = params.toList();
	this->can_data_pub.publish(msg);
}

/**
 * @brief send new order if the robot is too slow
 * @details we consider that the robot is blocked at the end of the timer
 * @param[in] timer timer event
 */
void ReachActionPerfomer::timeoutCallback(const ros::TimerEvent& timer){
	// Cancel action
	cancel();

	// Pause it
	returns(ActionStatus::PAUSED);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "reach");

	ReachActionPerfomer performer("reach");
	ros::spin();

	return 0;
}
