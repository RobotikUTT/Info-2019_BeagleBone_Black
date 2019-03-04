#ifndef INPUT_SIMULATION_NODE
#define INPUT_SIMULATION_NODE

#include <ros/ros.h>

#include "node_watcher/NodeStatusHandler.hpp"

#include "ai_msgs/NodeStatus.h"

using ai_msgs::NodeStatus;

class InputSimulationNode {
private:
	NodeStatusHandler nodes;

public:
	InputSimulationNode();
};

#endif