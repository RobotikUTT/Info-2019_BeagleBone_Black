#include "controller/controller_node.hpp"

/**
 * @brief Constructs the object.
 *
 * @param n	NodeHandle var
 * 
 */
Controller::Controller() : Node("controller", "ai"), side(Side::DOWN) {
	// attributes
	direction = Directions::NONE;
	proximity_stop = false;
	panelUp = 0;
	startSignalReceived = false;
	robotState = RobotStatus::ROBOT_INIT;

	// Advertisers
	proximity_stop_pub = nh.advertise<ProximityStop>("proximity", 1);
	can_data_pub = nh.advertise<interface_msgs::CanData>("/can_interface/out", 1);
	robot_status_pub = nh.advertise<RobotStatus>("/ai/controller/robot_status", 1);

	// Subscribers
	can_data_sub = nh.subscribe("/can_interface/in", 1, &Controller::onCanData, this);
	
	start_sub = nh.subscribe("/signal/start", 1, &Controller::onStartSignal, this);
	
	schedulerController = nh.serviceClient<SetSchedulerState>("/scheduler/do");

	// Wait for required nodes
	waitForNodes(30);
}

void Controller::onWaitingResult(bool success) {
	if (success) {
		// If start signal was received during waiting
		this->robotState = RobotStatus::ROBOT_READY;

		// Set as ready
		setNodeStatus(NodeStatus::READY);

		start();
	} else {
		setNodeStatus(NodeStatus::ERROR);
	}
}

void Controller::onStartSignal(const StartRobot& msg) {
	this->startSignalReceived = true;
	this->side = msg.side;
	start();
}

void Controller::start() {
	// If the robot is not ready or is not supposed to start
	if (!this->startSignalReceived ||
		this->robotState != RobotStatus::ROBOT_READY) {
		return;
	}

	// init STM position
	Argumentable params;
	params.setLong("x", 0);
	params.setLong("y", 0);
	params.setLong("angle", 0);

	interface_msgs::CanData msg;
	msg.type = "set_position";
	msg.params = params.toList();
	this->can_data_pub.publish(msg);

	// make it running
	params.reset();
	params.setLong("mode", interface_msgs::StmMode::START);

	msg.type = "set_stm_mode";
	msg.params = params.toList();
	this->can_data_pub.publish(msg);

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
	Argumentable params;
	interface_msgs::CanData msg;
	msg.type = "set_stm_mode";

	params.setLong("mode", interface_msgs::StmMode::STOP);
	msg.params = params.toList();
	this->can_data_pub.publish(msg);

	params.setLong("mode", interface_msgs::StmMode::RESET_ORDERS);
	msg.params = params.toList();
	this->can_data_pub.publish(msg);
}

void Controller::onCanData(const interface_msgs::CanData::ConstPtr& msg) {
	Argumentable input;
	input.fromList(msg->params);

	// TODO can interface to check for channels
	if (msg->type == "current_speed") {
		// Set robot direction according to speed
		int16_t linearSpeed = input.getLong("linear_speed");

		if (linearSpeed > 0) {
			direction = Directions::FORWARD;
		} else if (linearSpeed < 0) {
			direction = Directions::BACKWARD;
		} else {
			direction = Directions::NONE;
		}
	}

	else if (msg->type == "sonar_distance") {
		this->processSonars(input);
	}
}

/**
 * process sonars data to detect if an proximity stop is
 * mandatory
 */
void Controller::processSonars(const Argumentable& data) {
	bool last_proximity_value = proximity_stop;
	uint8_t front_left, front_right,
		back_left, back_right;

	front_left = data.getLong("dist_front_left");
	front_right = data.getLong("dist_front_right");
	back_left = data.getLong("dist_back_left");
	back_right = data.getLong("dist_back_right");

	/*
	 * Proximity stop is enabled in case the distance
	 * become less or equals than limit distances in
	 * the current direction.
	 */
	bool forward_stop = direction == Directions::FORWARD && (
		front_left <= SONAR_MIN_DIST_FORWARD + 6 ||
		front_right <= SONAR_MIN_DIST_FORWARD + 16
	);
	bool backward_stop = direction == Directions::BACKWARD && (
		back_left <= SONAR_MIN_DIST_BACKWARD ||
		back_right <= SONAR_MIN_DIST_BACKWARD
	);

	proximity_stop = forward_stop || backward_stop;

	// Declare unknown shape
	if (forward_stop) {
		// TODO
	} else {
		// TODO
	}

	if (last_proximity_value != proximity_stop) {
		ProximityStop proximity_msg;
		proximity_msg.proximity_set = proximity_stop;
		proximity_stop_pub.publish(proximity_msg);

		if (proximity_stop) {
			ROS_WARN("Set proximity stop");
		} else {
			ROS_WARN("Unset proximity stop");
		}

		interface_msgs::CanData msg;
		msg.type = "set_stm_mode";

		Argumentable params;
		params.setLong("mode",
			proximity_stop ? interface_msgs::StmMode::SETEMERGENCYSTOP : interface_msgs::StmMode::UNSETEMERGENCYSTOP);

		msg.params = params.toList();
		this->can_data_pub.publish(msg);
	}
}


int main(int argc, char *argv[]) {
	ros::init(argc, argv, "controller_node");
	Controller node;
	ros::spin();
}
