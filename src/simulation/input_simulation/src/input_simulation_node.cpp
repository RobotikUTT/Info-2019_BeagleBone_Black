#include "input_simulation/input_simulation_node.hpp"

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "input_simulation_node");
	InputSimulationNode node;
	ros::spin();
}
