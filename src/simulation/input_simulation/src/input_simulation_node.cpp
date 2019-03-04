#include "input_simulation/input_simulation_node.hpp"

InputSimulationNode::InputSimulationNode() : nodes(), nh() {
	// Init publishers and receivers
	this->startSigPub = nh.advertise<ai_msgs::StartRobot>(Topics::START_SIGNAL_TOPIC, 1);

	// Simule ros_can interface
	nodes.setNodeStatus("interface", "ros_can", NodeStatus::NODE_READY);

	// Wait for controller and scheduler
	nodes.require("/ai/controller");
	nodes.require("/ai/scheduler");

	nodes.setWaitCallback(boost::bind(&InputSimulationNode::nodesReady, this, _1));
	nodes.waitForNodes(6);
}

void InputSimulationNode::nodesReady(bool success) {
	// Scheduler and controller ready, send start signal !
	if (!success) {
		ROS_ERROR_STREAM("Controller or scheduler did not woke up...");
		return;
	}

	ai_msgs::StartRobot msg;
	msg.side = ai_msgs::Side::LEFT;

	this->startSigPub.publish(msg);
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "input_simulation_node");
	InputSimulationNode node;
	ros::spin();
}
