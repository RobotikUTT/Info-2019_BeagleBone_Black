#include "controller/controller_node.hpp"

/**
 * @brief Constructs the object.
 *
 * @param n	NodeHandle var
 * 
 */
Controller::Controller() : Node("controller", "ai"), side(Side::DOWN) {
	// attributes
	done = false;
	startSignalReceived = false;
	robotState = RobotStatus::ROBOT_INIT;

	// Advertisers
	can_data_pub = nh.advertise<interface_msgs::CanData>("/can_interface/out", 1);

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

	// Set positision
	Argumentable params;
	params.setLong("x", srv.response.point.x);
	params.setLong("y", srv.response.point.y);
	params.setLong("angle", srv.response.point.theta);
	interface_msgs::CanData msg;
	msg.type = "set_position";
	msg.params = params.toList();
	this->can_data_pub.publish(msg);

	ros::Duration(0.3).sleep();

	// make it running
	params.reset();
	params.setLong("mode", interface_msgs::StmMode::START);

	interface_msgs::CanData msg2;
	msg2.type = "set_stm_mode";
	msg2.params = params.toList();
	this->can_data_pub.publish(msg2);

	ros::Duration(0.3).sleep();

	// set left pid
	interface_msgs::CanData msg3;
	params.reset();
	params.setLong("p", 240);
	params.setLong("i", 0);
	params.setLong("d", 20000);
	msg3.type = "set_left_pid";
	msg3.params = params.toList();
	this->can_data_pub.publish(msg3);

	ros::Duration(0.3).sleep();

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

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "controller_node");
	Controller node;
	ros::spin();
}
