#ifndef INPUT_SIMULATION_NODE
#define INPUT_SIMULATION_NODE

#include <ros/ros.h>

#include "node_watcher/NodeStatusHandler.hpp"

#include "ai_msgs/NodeStatus.h"
#include "ai_msgs/Topics.h"
#include "ai_msgs/StartRobot.h"
#include "ai_msgs/Side.h"

using ai_msgs::NodeStatus;
using ros::Publisher;

class InputSimulationNode {
private:
	NodeStatusHandler nodes;
	ros::NodeHandle nh;

	Publisher startSigPub;

	void nodesReady(bool success);
public:
	InputSimulationNode();

protected:
};

#endif