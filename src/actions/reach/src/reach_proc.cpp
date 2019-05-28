#include "reach/reach_proc.hpp"


/**
 * @brief object constructor
 *
 * @param[in] name name of the action server
 */
ReachActionPerformer::ReachActionPerformer(std::string name) : ActionPerformer(name) {
	this->can_data_sub = nh.subscribe("/can_interface/in", 1, &ReachActionPerformer::onCanData, this);
	this->can_data_pub = nh.advertise<interface_msgs::CanData>("/can_interface/out", 1);

	this->get_sided_point_srv = nh.serviceClient<ai_msgs::GetSidedPoint>("/ai/map_handler/get_sided_point");

	if (!nh.getParam("/pathfinder/enabled", this->usePathfinder)) {
		ROS_WARN_STREAM("unable to determine whether pathfinder is enabled or not, disabling by default");
		this->usePathfinder = false;
	}

	if (this->usePathfinder) {
		this->pathfinder_srv = nh.serviceClient<pathfinder::FindPath>("/ai/pathfinder/findpath");
		this->require("/ai/pathfinder");
		this->waitForNodes(10, false);
	} else {
		this->onWaitingResult(true);
	}
}

void ReachActionPerformer::onWaitingResult(bool result) {
	if (result) {
		setNodeStatus(NodeStatus::READY);
	} else {
		ROS_ERROR_STREAM("unable to start reach action, missing pathfinder...");
		setNodeStatus(NodeStatus::ERROR);
	}
}

ActionPoint ReachActionPerformer::computeActionPoint(Argumentable* actionArgs, Pose2D robotPos) {
	ActionPoint result;
	result.start = robotPos;
	result.end.x = getLong("x", robotPos.x);
	result.end.y = getLong("y", robotPos.y);
	result.end.theta = getLong("angle", robotPos.theta);

	ai_msgs::GetSidedPoint srv;
	srv.request.point = result.end;
	srv.request.side = actionArgs->getLong("_side");
	
	ros::service::waitForService("/ai/map_handler/get_sided_point", 2);
	if (this->get_sided_point_srv.call(srv)) {
		result.end = srv.response.point;
	} else {
		ROS_ERROR_STREAM("Unable to get sided point for action point computation");
	}

	return result;
}

/**
 * @brief Callback called when the STM finished all move order
 * @param[in]  msg   The finish message
 */
void ReachActionPerformer::onCanData(const interface_msgs::CanData::ConstPtr& msg){
	if (msg->type == "order_complete") {
		timerTimeout.stop();
		returns(ActionStatus::DONE);

	} else if (msg->type == "current_pos") {
		Argumentable input;
		input.fromList(msg->params);

		this->robotPos.x = input.getLong("x");
		this->robotPos.y = input.getLong("y");
		this->robotPos.theta = this->convertAngle(input.getLong("angle"));
	}
}

/**
 * @brief run action toward a new goal and send the appropriate to the STM
 */
void ReachActionPerformer::start() {
	// Test that some actions are to be performed
	if (hasLong("x") * hasLong("y") + 2 * hasLong("angle") == 0) {
		ROS_ERROR_STREAM("missing data in message, need at least (x,y) or (angle)");
		returns(ActionStatus::ERROR);
		return;
	}
	
	// Get destination
	geometry_msgs::Pose2D posEnd;
	posEnd.x = getLong("x", 0);
	posEnd.y = getLong("y", 0);
	// Convert from degree to mrad
	posEnd.theta = this->convertAngle(getLong("angle", 0));

	// Get point from right side
	ai_msgs::GetSidedPoint srv;
	srv.request.point = posEnd;
	srv.request.side = getLong("_side");
	
	ros::service::waitForService("/ai/map_handler/get_sided_point", 2);
	if (this->get_sided_point_srv.call(srv)) {
		posEnd = srv.response.point;
	} else {
		ROS_ERROR_STREAM("Unable to get sided point for action point computation");
	}

	// If require movement
	if (hasLong("x") && hasLong("y")) {
		// Pathfinder?
		if (this->usePathfinder) {
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
					this->moveTo(pose, "go_to");
				}
			} else {
				ROS_ERROR_STREAM("Error while calling pathfinder service");
				this->returns(ActionStatus::ERROR);
			}
		} else {
			// x,y and maybe angle provided
			this->moveTo(posEnd, hasLong("angle") ? "go_to_angle" : "go_to");
		}
	} else if (hasLong("angle")) {
		// Only angle provided
		this->moveTo(posEnd, "rotate");
	} else {
		ROS_ERROR_STREAM("Missing arguments for [reach], provide x/y and/or angle.");
	}

	int timeout = getLong("timeout", 0);
	if (timeout > 0) {
		timerTimeout = nh.createTimer(ros::Duration(timeout), &ReachActionPerformer::timeoutCallback , this, true);
	}
}

unsigned long ReachActionPerformer::convertAngle(long degree) const {
	// Convert from degree to mrad
	double fraction = ((degree % 360 + 360) % 360);
	fraction /= 360;
	return fraction * 1000 * 2 * M_PI;
}

void ReachActionPerformer::moveTo(geometry_msgs::Pose2D location, std::string request) {
	Argumentable params;
	params.setLong("x", location.x);
	params.setLong("y", location.y);
	params.setLong("angle", location.theta);
	params.setLong("direction", interface_msgs::Directions::FORWARD);

	interface_msgs::CanData msg;
	msg.type = request;
	msg.params = params.toList();

	this->can_data_pub.publish(msg);
}

void ReachActionPerformer::cancel() {
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
void ReachActionPerformer::timeoutCallback(const ros::TimerEvent& timer){
	// Cancel action
	cancel();

	// Pause it
	returns(ActionStatus::PAUSED);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "reach");

	ReachActionPerformer performer("reach");
	ros::spin();

	return 0;
}
