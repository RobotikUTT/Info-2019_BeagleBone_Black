#include "controller/controller_node.hpp"

/**
 * @brief Constructs the object.
 *
 * @param n	NodeHandle var
 * 
 */
Controller::Controller() : Node("controller", "ai"), side(Side::DOWN) {
	// attributes
	direction = Directions::FORWARD;
	proximity_stop = false;
	done = false;
	panelUp = 0;
	startSignalReceived = false;
	robotState = RobotStatus::ROBOT_INIT;

	// Advertisers
	can_data_pub = nh.advertise<interface_msgs::CanData>("/can_interface/out", 1);

	// Subscribers
	can_data_sub = nh.subscribe("/can_interface/in", 1, &Controller::onCanData, this);
	
	start_sub = nh.subscribe("/signal/start", 1, &Controller::onStartSignal, this);
	
	schedulerController = nh.serviceClient<SetSchedulerState>("/scheduler/do");
	sidedPoint = nh.serviceClient<GetSidedPoint>("/ai/map_handler/get_sided_point");

	// Wait for required nodes
	waitForNodes(3000);
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

	int timeout = 100;
	if (!nh.getParam("/actions/timeout", timeout)) {
		ROS_ERROR_STREAM("No /actions/timeout defined in params, using 100");
		timeout = 100;

	}
	timer = nh.createTimer(ros::Duration(timeout), &Controller::stop, this, true);

	// init STM position
	ai_msgs::GetSidedPoint srv;
	srv.request.side = this->side;
	bool found = nh.getParam("/robot/start/x", srv.request.point.x) &&
		nh.getParam("/robot/start/y", srv.request.point.y) &&
		nh.getParam("/robot/start/angle", srv.request.point.theta);

	// Convert angle
	double angleRad = (long)(srv.request.point.theta + 360) % 360;
	angleRad /= 360;
	srv.request.point.theta = angleRad * 1000 * M_PI;

	if (!found) {
		ROS_ERROR_STREAM("unable to get x, y and angle parameters for initial robot position");
	}

	if (!this->sidedPoint.call(srv)) {
		ROS_ERROR_STREAM("Unable to get point with correct side reference");
		setNodeStatus(NodeStatus::ERROR);
		return;
	}

	Argumentable params;
	params.setLong("x", srv.response.point.x);
	params.setLong("y", srv.response.point.y);
	params.setLong("angle", srv.response.point.theta);

	interface_msgs::CanData msg;
	msg.type = "set_position";
	msg.params = params.toList();
	this->can_data_pub.publish(msg);

	// make it running
	params.reset();
	params.setLong("mode", interface_msgs::StmMode::START);

	interface_msgs::CanData msg2;
	msg2.type = "set_stm_mode";
	msg2.params = params.toList();
	this->can_data_pub.publish(msg2);

	// set left pid
	interface_msgs::CanData msg3;
	params.reset();
	params.setLong("p", 240);
	params.setLong("i", 0);
	params.setLong("d", 20000);
	msg3.type = "set_left_pid";
	msg3.params = params.toList();
	this->can_data_pub.publish(msg3);

	// set right pid
	params.setLong("p", 130);
	interface_msgs::CanData msg4;
	msg4.type = "set_right_pid";
	msg4.params = params.toList();
	this->can_data_pub.publish(msg4);


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
void Controller::stop(const ros::TimerEvent& timer) {
	// stop actions by calling scheduler service
	SetSchedulerState setter;
	setter.request.running = false;
	schedulerController.call(setter);

	// stop all movements and actions
	Argumentable params;
	interface_msgs::CanData msg;
	msg.type = "set_stm_mode";

	params.setLong("mode", interface_msgs::StmMode::RESET_ORDERS);
	msg.params = params.toList();
	this->can_data_pub.publish(msg);

	interface_msgs::CanData msg2;
	msg2.type = "set_stm_mode";
	params.setLong("mode", interface_msgs::StmMode::STOP);
	msg2.params = params.toList();
	this->can_data_pub.publish(msg2);
}

void Controller::onCanData(const interface_msgs::CanData::ConstPtr& msg) {
	Argumentable input;
	input.fromList(msg->params);

	// TODO can interface to check for channels
	if (msg->type == "current_speed") {
		// Set robot direction according to speed
		// Exceeding values allow to go back
		int16_t linearSpeed = input.getLong("linear_speed");

		if (linearSpeed > 0) {
			direction = Directions::FORWARD;
		} else if (linearSpeed < 0) {
			direction = Directions::BACKWARD;
		} else if (!this->proximity_stop) {
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
	bool last_proximity_value = this->proximity_stop;

	uint8_t front_left = data.getLong("dist_front_left", 255);
	uint8_t front_right = data.getLong("dist_front_right", 255);
	uint8_t back_left = data.getLong("dist_back_left", 255);
	uint8_t back_right = data.getLong("dist_back_right", 255);


	/*
	 * Proximity stop is enabled in case the distance
	 * become less or equals than limit distances in
	 * the current direction.
	 */
	bool forward_stop = direction == Directions::FORWARD && (
		front_left <= SONAR_MIN_DIST_FORWARD + (this->proximity_stop ? 10 : 0) ||
		front_right <= SONAR_MIN_DIST_FORWARD + (this->proximity_stop ? 10 : 0)
	);
	bool backward_stop = direction == Directions::BACKWARD && (
		back_left <= SONAR_MIN_DIST_BACKWARD + (this->proximity_stop ? 10 : 0) ||
		back_right <= SONAR_MIN_DIST_BACKWARD + (this->proximity_stop ? 10 : 0)
	);

	this->proximity_stop = forward_stop || backward_stop;

	// Declare unknown shape
	if (forward_stop) {
		// TODO
	} else {
		// TODO
	}

	if (last_proximity_value != this->proximity_stop) {
		if (this->proximity_stop) {
			ROS_WARN("Set proximity stop");
		} else {
			ROS_WARN("Unset proximity stop");
		}

		interface_msgs::CanData msg;
		msg.type = "set_stm_mode";

		Argumentable params;
		params.setLong("mode",
			this->proximity_stop ? interface_msgs::StmMode::SETEMERGENCYSTOP : interface_msgs::StmMode::UNSETEMERGENCYSTOP);

		msg.params = params.toList();
		this->can_data_pub.publish(msg);
	}
}


int main(int argc, char *argv[]) {
	ros::init(argc, argv, "controller_node");
	Controller node;
	ros::spin();
}
