/** @file controller_node.cpp
*		@brief Node class wich controls all other nodes other robot watcher.
*		
*		@author Alexis CARE
*/

#include "controller/controller_node.hpp"

//define typedef for lisibility
typedef boost::shared_ptr<::procedures_msgs::MoveResult const> MoveResultConstPtr;

/**
 * @brief Constructs the object.
 *
 * @param n	NodeHandle var
 * 
 */
Controller::Controller() : Node("controller", "ai") {
	// attributes
	side = SIDE_GREEN;
	direction = NONE;
	emergency_stop = false;
	panelUp = 0;
	started = false;
	robot_state = ROBOT_INIT;

	// Advertisers
	emergency_stop_pub = nh.advertise<ai_msgs::EmergencyStop>("emergency", 1);
	STM_SetPose_pub = nh.advertise<can_msgs::Point>("/STM/SetPose", 1);
	STM_AsserManagement_pub = nh.advertise<can_msgs::Status>("/STM/AsserManagement", 1);


	// Subscribers
	status_sub = nh.subscribe("robot_watcher/robot_status", 1, &Controller::setRobotStatus, this);
	robot_pos_sub = nh.subscribe("/STM/Position", 1, &Controller::setRobotPosition, this);
	robot_speed_sub = nh.subscribe("/STM/GetSpeed", 1, &Controller::setRobotSpeed, this);
	sonar_distance_sub = nh.subscribe("/ARDUINO/SonarDistance", 1, &Controller::processSonars, this);
	robot_blocked_sub = nh.subscribe("/STM/RobotBlocked", 1, &Controller::processRobotBlocked, this);
	
	side_sub = nh.subscribe("side", 1, &Controller::setSide, this);
	start_sub = nh.subscribe("start", 1, &Controller::onStartSignal, this);
	
	schedulerController = nh.serviceClient<ai_msgs::SetSchedulerState>("/scheduler/do");

	setNodeStatus(NODE_READY);
}

void Controller::onStartSignal(const std_msgs::Empty& msg) {
	started = true;

	// If the scheduler is ready to go (it wait itself for required actions)
	if (robot_state == NODE_READY) {
		// We can start it !
		ai_msgs::SetSchedulerState setter;
		setter.request.running = true;
		schedulerController.call(setter);
	}
}

void Controller::setSide(const ai_msgs::SetSide::ConstPtr& msg) {
	side = msg->side;
}

// Set robot position inside controller
void Controller::setRobotPosition(const can_msgs::Point::ConstPtr& msg) {
	// ROS_INFO_STREAM("robot pos_x: " << msg->pos_x
	//	<< " robot pos_y: " << msg->pos_y
	//	<< " robot angle: " << msg->angle);
	robot_pos_x = msg->pos_x;
	robot_pos_y = msg->pos_y;
	robot_angle = msg->angle;
}

/**
 * @brief Gets the robot status and act accordingly
 * - ROBOT_READY and ROBOT_INIT : do nothing and wait for it to be running
 * - ROBOT_RUNNING : wake up the robot run actions
 * - ROBOT_HALT : stop all actions
 *
 * @param[in]	msg	 The RobotStatus message
 */
void Controller::setRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg) {
	uint8_t robot_status = msg->robot_status;

	if (robot_status == ROBOT_READY) {
		// ??? unused state here
	} else if (robot_status == ROBOT_RUNNING) {
		// ROS_DEBUG("Robot Running");

		// Compute initial position
		int x, y, angle;
		nh.getParam("controller/robot_pos/x", x);
		nh.getParam("controller/robot_pos/y", y);
		nh.getParam("controller/robot_pos/angle", angle);

		if (side) { // if orange
			y = (1500 - y) + 1500;
			angle = -angle;
		}

		// init STM position
		can_msgs::Point msg;
		robot_pos_x = x;
		robot_pos_y = y;
		robot_angle = angle;
		msg.pos_x = x;
		msg.pos_y = y;
		msg.angle = angle;
		STM_SetPose_pub.publish(msg);

		// make it running
		can_msgs::Status msg2;
		msg2.value = START;
		STM_AsserManagement_pub.publish(msg2);

		// start actions by calling scheduler service
		ai_msgs::SetSchedulerState setter;
		setter.request.running = true;
		schedulerController.call(setter);

	} else if (robot_status == ROBOT_HALT) {
		// stop actions by calling scheduler service
		ai_msgs::SetSchedulerState setter;
		setter.request.running = false;
		schedulerController.call(setter);

		// stop all movements and actions
		can_msgs::Status msg;
		msg.value = STOP;
		STM_AsserManagement_pub.publish(msg);

		msg.value = RESET_ORDERS;
		STM_AsserManagement_pub.publish(msg);
	}
}

/**
 * @brief check if the LED panel is connected
 *
 * @param[in]	msg	 The NodesStatus message
 */
/*[[deprecated("panels and points should be handled in a separate package")]]
void Controller::checkForPanel(const ai_msgs::NodesStatus::ConstPtr &msg) {
	if (getNodeStatus("PANEL", "board").status == NODE_READY) {
		panelUp = 1;
	}
}*/

/**
 * @brief retrieve robot speed from can
 *
 * @param[in] msg message containing current speed
 */
void Controller::setRobotSpeed(const can_msgs::CurrSpeed::ConstPtr &msg) {
	int16_t linearSpeed = msg->linear_speed;
	int16_t leftSpeed = msg->left_speed;
	int16_t rightSpeed = msg->right_speed;

	//	ROS_INFO_STREAM("SPEEDS|linear: " << linearSpeed
	//	<< " left: " << leftSpeed
	//	<< " right: " << rightSpeed);

	if (linearSpeed > 0) {
		direction = FORWARD;
	}
	else if (linearSpeed < 0) {
		direction = BACKWARD;
	}
	else {
		direction = NONE;
	}
}

/**
 * @brief process sonars data to detect if an emergency stop is
 * mandatory
 *
 * @param[in] msg message containing sonars data
 */
void Controller::processSonars(const can_msgs::SonarDistance::ConstPtr &msg) {
	bool last_emergency_value = emergency_stop;
	uint8_t front_left, front_right,
		back_left, back_right;

	front_left = msg->dist_front_left;
	front_right = msg->dist_front_right;
	back_left = msg->dist_back_left;
	back_right = msg->dist_back_right;

	// ROS_INFO_STREAM("DIST|" << front_left << "|" << front_right
	// << "|" << left << "|"	<< right << "|" << back);
	/*ROS_INFO("DIST|%u|%u|%u|%u|",front_left,
		front_right, back_left, back_right);*/

	/*
	 * Emergency stop is enabled in case the distance
	 * become less or equals than limit distances in
	 * the current direction.
	 */
	emergency_stop = (
		direction == FORWARD && (
			front_left <= SONAR_MIN_DIST_FORWARD + 6 ||
			front_right <= SONAR_MIN_DIST_FORWARD + 16
		)
	) || (
		direction == BACKWARD && (
			back_left <= SONAR_MIN_DIST_BACKWARD ||
			back_right <= SONAR_MIN_DIST_BACKWARD
		)
	);

	if (last_emergency_value != emergency_stop) {
		ai_msgs::EmergencyStop emergency_msg;
		emergency_msg.emergency_set = emergency_stop;
		emergency_stop_pub.publish(emergency_msg);

		if (emergency_stop) {
			ROS_WARN("SET EMG");
		} else {
			ROS_WARN("UNSET EMG");
		}

		can_msgs::Status can_msg;
		if (emergency_stop) {
			can_msg.value = SETEMERGENCYSTOP;
		}
		else {
			can_msg.value = UNSETEMERGENCYSTOP;
		}

		STM_AsserManagement_pub.publish(can_msg);
	}
}

/**
 * @brief callback to process a blocked robot
 *
 * @param[in] msg RobotBlocked message
 * 
 * @todo To dev
 */
void Controller::processRobotBlocked(const can_msgs::RobotBlocked::ConstPtr &msg) {
	ROS_WARN("Robot blocked");
}


int main(int argc, char *argv[]) {
	ros::init(argc, argv, "controller_node");
	Controller node;
	ros::spin();
}
