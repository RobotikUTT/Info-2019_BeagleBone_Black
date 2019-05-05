#include "reach/reach_proc.hpp"


/**
 * @brief object constructor
 *
 * @param[in] name name of the action server
 */
ReachActionPerfomer::ReachActionPerfomer(std::string name) : ActionPerformer(name) {
	this->can_data_sub = nh.subscribe("/can_interface/in", 1, &ReachActionPerfomer::onCanData, this);
	this->can_data_pub = nh.advertise<interface_msgs::CanData>("/can_interface/out", 1);

	setNodeStatus(NodeStatus::READY);
}

ActionPoint ReachActionPerfomer::computeActionPoint(Argumentable* actionArgs, Pose2D robotPos) {
	// TODO
	return ActionPoint();
}

/**
 * @brief Callback called when the STM finished all move order
 * @param[in]  msg   The finish message
 */
void ReachActionPerfomer::onCanData(const interface_msgs::CanData::ConstPtr& msg){
	if (msg->type == "order_complete") {
		timerTimeout.stop();
		actionPerformed();
	}
}

/**
 * @brief run action toward a new goal and send the appropriate to the STM
 */
void ReachActionPerfomer::start() {
	interface_msgs::CanData msg;
	Argumentable params;

	// Copy parameters
	params.setLong("x", getLong("x", 0));
	params.setLong("y", getLong("y", 0));
	params.setLong("angle", getLong("angle", 0));
	params.setLong("direction", getLong("direction", 0));

	/**
	 * Boolean equation determining which move the action should use
	 */
	int moveType = hasLong("x") * hasLong("y") * hasLong("direction") + 2 * hasLong("angle");

	switch (moveType) {
		case 1: // x, y and direction provided
			msg.type = "go_to";
			break;

		case 2: // angle only provided
			msg.type = "rotate";
			break;

		case 3: // everything is provided
			msg.type = "go_to_angle";
			break;

		default:
			ROS_ERROR_STREAM("unable to determine message type to use " << moveType << " like " << hasLong("x") << ":" << hasLong("y") << ":" << hasLong("direction") << ":" << hasLong("angle"));
			throw "unable to determine message type to use";
			break;
	}

	msg.params = params.toList();
	this->can_data_pub.publish(msg);

	int timeout = getLong("timeout", 0);
	if (timeout > 0) {
		timerTimeout = nh.createTimer(ros::Duration(timeout), &ReachActionPerfomer::timeoutCallback , this, true);
	}
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
	actionPaused();
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "reach");

	ReachActionPerfomer performer("reach");
	ros::spin();

	return 0;
}
