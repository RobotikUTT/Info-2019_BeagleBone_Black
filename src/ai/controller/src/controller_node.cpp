#include "controller/controller_node.hpp"

/**
 * @brief Constructs the object.
 *
 * @param n	NodeHandle var
 * 
 */
Controller::Controller() : Node("controller", "ai"), side(Side::LEFT) {
	// attributes
	direction = Point::DIRECTION_NONE;
	proximity_stop = false;
	panelUp = 0;
	startSignalReceived = false;
	robotState = RobotStatus::ROBOT_INIT;

	// Advertisers
	proximity_stop_pub = nh.advertise<ProximityStop>("proximity", 1);
	STM_SetPose_pub = nh.advertise<can_msgs::Point>("/STM/SetPose", 1);
	STM_AsserManagement_pub = nh.advertise<can_msgs::STMStatus>("/STM/AsserManagement", 1);
	robot_status_pub = nh.advertise<RobotStatus>("/ai/controller/robot_status", 1);

	// Subscribers
	robot_speed_sub = nh.subscribe("/STM/GetSpeed", 1, &Controller::setRobotSpeed, this);
	sonar_distance_sub = nh.subscribe("/ARDUINO/SonarDistance", 1, &Controller::processSonars, this);
	
	start_sub = nh.subscribe("/signal/start", 1, &Controller::onStartSignal, this);
	
	schedulerController = nh.serviceClient<SetSchedulerState>("/scheduler/do");

	// Wait for required nodes
	if (waitForNodes(2)) {
		// Set as ready
		setNodeStatus(NodeStatus::NODE_READY);

		// If start signal was received during waiting
		this->robotState = RobotStatus::ROBOT_READY;
		start();
	} else {
		setNodeStatus(NodeStatus::NODE_ERROR);
	}
	
}

void Controller::onStartSignal(const StartRobot& msg) {
	this->startSignalReceived = true;
	this->side = msg.side;
	start();
}

void Controller::start() {
	// Compute initial position
	int x, y, angle;
	nh.getParam("controller/robot_pos/x", x);
	nh.getParam("controller/robot_pos/y", y);
	nh.getParam("controller/robot_pos/angle", angle);

	// init STM position
	can_msgs::Point msg;
	msg.pos_x = x;
	msg.pos_y = y;
	msg.angle = angle;
	STM_SetPose_pub.publish(msg);

	// make it running
	can_msgs::STMStatus msg2;
	msg2.value = can_msgs::STMStatus::START;
	STM_AsserManagement_pub.publish(msg2);

	// start actions by calling scheduler service
	SetSchedulerState setter;
	setter.request.running = true;
	setter.request.side = this->side;
	schedulerController.call(setter);

	this->robotState = RobotStatus::ROBOT_RUNNING;
}

/**
 * @brief stop all actions
 * @param[in]	msg	 The RobotStatus message
 */
void Controller::stop() {
	// stop actions by calling scheduler service
	SetSchedulerState setter;
	setter.request.running = false;
	schedulerController.call(setter);

	// stop all movements and actions
	can_msgs::STMStatus msg;
	msg.value = can_msgs::STMStatus::STOP;
	STM_AsserManagement_pub.publish(msg);

	msg.value = can_msgs::STMStatus::RESET_ORDERS;
	STM_AsserManagement_pub.publish(msg);
}

/**
 * @brief retrieve robot speed from can
 * @param[in] msg message containing current speed
 */
void Controller::setRobotSpeed(const can_msgs::CurrSpeed::ConstPtr &msg) {
	int16_t linearSpeed = msg->linear_speed;
	int16_t leftSpeed = msg->left_speed;
	int16_t rightSpeed = msg->right_speed;

	if (linearSpeed > 0) {
		direction = Point::DIRECTION_FORWARD;
	}
	else if (linearSpeed < 0) {
		direction = Point::DIRECTION_BACKWARD;
	}
	else {
		direction = Point::DIRECTION_NONE;
	}
}

/**
 * @brief process sonars data to detect if an proximity stop is
 * mandatory
 *
 * @param[in] msg message containing sonars data
 */
void Controller::processSonars(const can_msgs::SonarDistance::ConstPtr &msg) {
	bool last_proximity_value = proximity_stop;
	uint8_t front_left, front_right,
		back_left, back_right;

	front_left = msg->dist_front_left;
	front_right = msg->dist_front_right;
	back_left = msg->dist_back_left;
	back_right = msg->dist_back_right;
	/*
	 * Proximity stop is enabled in case the distance
	 * become less or equals than limit distances in
	 * the current direction.
	 */
	proximity_stop = (
		direction == Point::DIRECTION_FORWARD && (
			front_left <= SONAR_MIN_DIST_FORWARD + 6 ||
			front_right <= SONAR_MIN_DIST_FORWARD + 16
		)
	) || (
		direction == Point::DIRECTION_BACKWARD && (
			back_left <= SONAR_MIN_DIST_BACKWARD ||
			back_right <= SONAR_MIN_DIST_BACKWARD
		)
	);

	if (last_proximity_value != proximity_stop) {
		ProximityStop proximity_msg;
		proximity_msg.proximity_set = proximity_stop;
		proximity_stop_pub.publish(proximity_msg);

		if (proximity_stop) {
			ROS_WARN("SET EMG");
		} else {
			ROS_WARN("UNSET EMG");
		}

		can_msgs::STMStatus can_msg;
		if (proximity_stop) {
			can_msg.value = can_msgs::STMStatus::SETEMERGENCYSTOP;
		} else {
			can_msg.value = can_msgs::STMStatus::UNSETEMERGENCYSTOP;
		}

		STM_AsserManagement_pub.publish(can_msg);
	}
}


int main(int argc, char *argv[]) {
	ros::init(argc, argv, "controller_node");
	Controller node;
	ros::spin();
}
