#include "input_simulation/input_simulation_node.hpp"

InputSimulationNode::InputSimulationNode() : nodes() {
	// Simule ros_can interface
	nodes.setNodeStatus("interface", "ros_can", NodeStatus::NODE_READY);

	// Send start message
	nodes.require("controller", "ai");
	nodes.require("scheduler", "ai");
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "input_simulation_node");
	InputSimulationNode node;
	ros::spin();
}
