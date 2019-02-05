#include "reach/reach_proc.hpp"


/**
 * @brief object constructor
 *
 * @param[in] name name of the action server
 */
ReachActionPerfomer::ReachActionPerfomer(std::string name) : ActionPerformer(name) {
	finish_sub = nh.subscribe("/ALL/Finish", 1, &ReachActionPerfomer::analysisCB, this);
	
	this->STMGoToAngle_pub = nh.advertise<can_msgs::Point>("/STM/GoToAngle", 1);
	this->STMGoTo_pub = nh.advertise<can_msgs::Point>("/STM/GoTo", 1);
	this->STMRotation_pub = nh.advertise<can_msgs::Point>("/STM/Rotation", 1);
	this->STM_AsserManagement_pub = nh.advertise<can_msgs::Status>("/STM/AsserManagement", 1);

	ready();
}

ActionPoint* ReachActionPerfomer::computeActionPoint(std::vector<ai_msgs::Argument> actionArgs, procedures_msgs::OrPoint robot_pos) {
	// TODO
	return new ActionPoint();
}

/**
 * @brief      Callback called when the STM finished all move order
 *
 * @param[in]  msg   The finish message
 */
void ReachActionPerfomer::analysisCB(const can_msgs::Finish::ConstPtr& msg){
	// ROS_WARN_STREAM("Move; FINISH : state "<< act.isActive());

	if (/*!actionServer->isActive() ||*/ msg->val != 0)
		return;

	TimerTimeout.stop();

	actionPerformed();
}

/**
 * @brief run action toward a new goal and send the appropriate to the STM
 */
void ReachActionPerfomer::start() {
	can_msgs::Point msg;

	msg.pos_x = getArg("x", 0);
	msg.pos_y = getArg("y", 0);
	msg.angle = getArg("angle", 0);
	msg.direction = getArg("direction", 0);

	/**
	 * Boolean equation determining which move the action should use
	 */
	int moveType = hasArg("x") * hasArg("y") * hasArg("direction") + 2 * hasArg("angle");

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
			throw "unable to determine message type to use";
			break;
	}

	[[deprecated("timeout should be handled by actionperformer himself")]]
	{
		int timeout = getArg("timeout", 0);
		if (timeout > 0) {
			TimerTimeout = nh.createTimer(ros::Duration(timeout), &ReachActionPerfomer::TimeoutCallback , this, true);
		}
	}
}

/**
 * @brief send new order if the robot is too slow
 * 
 * @details we consider that the robot is blocked at the end of the timer 
 *
 * @param[in] timer timer event
 */
void ReachActionPerfomer::TimeoutCallback(const ros::TimerEvent& timer){
	// reset all goal in the STM
	can_msgs::Status msg;

	msg.value = RESET_ORDERS;
	STM_AsserManagement_pub.publish(msg);

	actionPerformed();

}


int main(int argc, char** argv) {
	ros::init(argc, argv, "reach_procedure");

	ReachActionPerfomer performer(ros::this_node::getName());
	ros::spin();

	return 0;
}
