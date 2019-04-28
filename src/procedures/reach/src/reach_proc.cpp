#include "reach/reach_proc.hpp"


/**
 * @brief object constructor
 *
 * @param[in] name name of the action server
 */
ReachActionPerfomer::ReachActionPerfomer(std::string name) : ActionPerformer(name) {
	finish_sub = nh.subscribe("/ALL/Finish", 1, &ReachActionPerfomer::movementDone, this);
	
	this->STMGoToAngle_pub = nh.advertise<geometry_msgs::Pose2D>("/STM/GoToAngle", 1);
	this->STMGoTo_pub = nh.advertise<geometry_msgs::Pose2D>("/STM/GoTo", 1);
	this->STMRotation_pub = nh.advertise<geometry_msgs::Pose2D>("/STM/Rotation", 1);
	this->STM_AsserManagement_pub = nh.advertise<interface_msgs::StmMode>("/STM/AsserManagement", 1);

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
void ReachActionPerfomer::movementDone(const interface_msgs::StmDone::ConstPtr& msg){
	// ROS_WARN_STREAM("Move; FINISH : state "<< act.isActive());

	if (/*!actionServer->isActive() ||*/ msg->val != 0)
		return;

	timerTimeout.stop();

	actionPerformed();
}

/**
 * @brief run action toward a new goal and send the appropriate to the STM
 */
void ReachActionPerfomer::start() {
	interface_msgs::DirectedPose msg;

	msg.x = getLong("x", 0);
	msg.y = getLong("y", 0);
	msg.theta = getLong("angle", 0);
	msg.direction = getLong("direction", 0);

	/**
	 * Boolean equation determining which move the action should use
	 */
	int moveType = hasLong("x") * hasLong("y") * hasLong("direction") + 2 * hasLong("angle");

	switch (moveType)
	{
		case 1: // x, y and direction provided
			STMGoTo_pub.publish(msg);
			break;

		case 2: // angle only provided
			STMRotation_pub.publish(msg);
			break;

		case 3: // everything is provided
			STMGoToAngle_pub.publish(msg);
			break;

		default:
			ROS_ERROR_STREAM("unable to determine message type to use " << moveType << " like " << hasLong("x") << ":" << hasLong("y") << ":" << hasLong("direction") << ":" << hasLong("angle"));
			throw "unable to determine message type to use";
			break;
	}

	int timeout = getLong("timeout", 0);
	if (timeout > 0) {
		timerTimeout = nh.createTimer(ros::Duration(timeout), &ReachActionPerfomer::timeoutCallback , this, true);
	}
}

void ReachActionPerfomer::cancel() {
	// reset all goal in the STM
	interface_msgs::StmMode msg;

	msg.value = interface_msgs::StmMode::RESET_ORDERS;
	STM_AsserManagement_pub.publish(msg);
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
