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

	this->proximityStop = false;
	this->forwardStop = false;
	this->backwardStop = false;

	// Fecth parameters
	if (!nh.getParam("/pathfinder/enabled", this->usePathfinder)) {
		ROS_WARN_STREAM("unable to determine whether pathfinder is enabled or not, disabling by default");
		this->usePathfinder = false;
	}

	if (!nh.getParam("/sonars/back_trigger_distance", this->backTriggerDistance)) {
		ROS_WARN_STREAM("/sonars/back_trigger_distance is not set, using 25 cm as default");
		this->backTriggerDistance = 25;
	}

	if (!nh.getParam("/sonars/front_trigger_distance", this->frontTriggerDistance)) {
		ROS_WARN_STREAM("/sonars/front_trigger_distance is not set, using 25 cm as default");
		this->frontTriggerDistance = 25;
	}

	if (!nh.getParam("/sonars/escape_distance", this->escapeDistance)) {
		ROS_WARN_STREAM("/sonars/escape_distance is not set, using 200 mm as default");
		this->escapeDistance = 200;
	}

	if (!nh.getParam("/sonars/proximity_timeout", this->proximityTimeout)) {
		ROS_WARN_STREAM("/sonars/proximity_timeout is not set, using 2 seconds as default");
		this->proximityTimeout = 2;
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

/**
 * process sonars data to detect if an proximity stop is
 * mandatory
 */
void ReachActionPerformer::processSonars(const Argumentable& data) {
	bool last_proximity_value = this->proximityStop;

	uint8_t front_left = data.getLong("dist_front_left", 255);
	uint8_t front_right = data.getLong("dist_front_right", 255);
	uint8_t back_left = data.getLong("dist_back_left", 255);
	uint8_t back_right = data.getLong("dist_back_right", 255);
	
	// Check if something is close in front or in back
	this->forwardStop = front_left <= this->frontTriggerDistance + (this->proximityStop ? 7 : 0) ||
		front_right <= this->frontTriggerDistance + (this->proximityStop ? 7 : 0);
		
	this->backwardStop = back_left <= this->backTriggerDistance + (this->proximityStop ? 7 : 0) ||
		back_right <= this->backTriggerDistance + (this->proximityStop ? 7 : 0);

	// Stop if disturbance match with direction
	this->proximityStop = (direction == Directions::FORWARD && forwardStop) ||
		(direction == Directions::BACKWARD && backwardStop);

	if (last_proximity_value != this->proximityStop) {
		// Run some timer to move backward
		if (this->proximityStop) {
			ROS_WARN("Set proximity stop");
			timerProximity.stop();
			timerProximity = nh.createTimer(ros::Duration(this->proximityTimeout), &ReachActionPerformer::onProximityTimeout, this, true);
		} else {
			ROS_WARN("Unset proximity stop");
			timerProximity.stop();
		}

		interface_msgs::CanData msg;
		msg.type = "set_stm_mode";

		Argumentable params;
		params.setLong("mode",
			this->proximityStop ? interface_msgs::StmMode::SETEMERGENCYSTOP : interface_msgs::StmMode::UNSETEMERGENCYSTOP);

		msg.params = params.toList();
		this->can_data_pub.publish(msg);
	}
}

/**
 * Runned when proximity stop is set for too much time
 */
void ReachActionPerformer::onProximityTimeout(const ros::TimerEvent& timer) {
	// reset all goal in the STM
	this->cancel();

	// If not locked from both sides
	if (!(this->forwardStop && this->backwardStop)) {
		geometry_msgs::Pose2D goal;
		ROS_INFO_STREAM(this->robotPos);
		// Get from mrad to rad
		double angleRad = this->robotPos.theta;
		angleRad /= 1000;
		ROS_INFO_STREAM(angleRad);
		// Values (for forward by default)
		int direction = 1;
		long dx = cos(angleRad) * this->escapeDistance;
		long dy = sin(angleRad) * this->escapeDistance;

		// Forward blocked
		if (this->forwardStop) {
			direction = 0;

			// Change direction
			dx = -dx;
			dy = -dy;
		}

		goal.x = dx + this->robotPos.x;
		goal.y = dy + this->robotPos.y;

		goal.x = goal.x > 0 ? goal.x : 0;
		goal.y = goal.y > 0 ? goal.y : 0;

		// Reset args to avoid further modificatio
		this->reset();

		// Start action as blocked and start handling again with new values
		this->blocked = true;
		this->moveTo(goal, "go_to", direction);
	} else {
		// Create new timer otherwise
		timerProximity = nh.createTimer(ros::Duration(this->proximityTimeout), &ReachActionPerformer::onProximityTimeout, this, true);
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
	Argumentable input;
	input.fromList(msg->params);

	if (msg->type == "order_complete") {
		timerTimeout.stop();
		timerProximity.stop();

		// If blocked pause action
		if (this->blocked) {
			returns(ActionStatus::PAUSED);
		} else {
			returns(ActionStatus::DONE);
		}

	}
	
	else if (msg->type == "current_pos") {
		this->robotPos.x = input.getLong("x");
		this->robotPos.y = input.getLong("y");
		this->robotPos.theta = input.getLong("angle");
	}
	
	else if (msg->type == "current_speed") {
		// Set robot direction according to speed
		// Exceeding values allow to go back
		int16_t linearSpeed = input.getLong("linear_speed");

		if (linearSpeed > 0) {
			direction = Directions::FORWARD;
		} else if (linearSpeed < 0) {
			direction = Directions::BACKWARD;
		} else if (!this->proximityStop) {
			direction = Directions::NONE;
		}
	}

	else if (msg->type == "sonar_distance") {
		this->processSonars(input);
	}
}


/**
 * @brief run action toward a new goal and send the appropriate to the STM
 */
void ReachActionPerformer::start() {
	// Reset blocked
	this->blocked = false;

	// Get direction
	int direction = getLong("direction", interface_msgs::Directions::FORWARD);

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
					this->moveTo(pose, "go_to", direction);
				}
			} else {
				ROS_ERROR_STREAM("Error while calling pathfinder service");
				this->returns(ActionStatus::ERROR);
			}
		} else {
			// x,y and maybe angle provided
			this->moveTo(posEnd, hasLong("angle") ? "go_to_angle" : "go_to", direction);
		}
	} else if (hasLong("angle")) {
		// Only angle provided
		this->moveTo(posEnd, "rotate", direction);
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

void ReachActionPerformer::moveTo(geometry_msgs::Pose2D location, std::string request, int direction) {
	Argumentable params;
	params.setLong("x", location.x);
	params.setLong("y", location.y);
	params.setLong("angle", location.theta);
	params.setLong("direction", direction);
	if (getLong("_side") == Side::UP && getString("change_direction_up_side", "false") == "true") {
		ROS_WARN_STREAM("Changing direction from " << direction << " to " << 1 - direction << " because up side.");
		params.setLong("direction", 1 - direction);
	}

	// Keep original angle if provided
	if (getString("same_angle", "false") == "true") {
		ROS_WARN_STREAM("Keeping original angle value " << this->convertAngle(getLong("angle", 0)) << ".");
		params.setLong("angle", this->convertAngle(getLong("angle", 0)));
	}

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
