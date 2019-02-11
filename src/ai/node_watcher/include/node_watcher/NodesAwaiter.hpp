#ifndef NODES_AWAITER_HPP
#define NODES_AWAITER_HPP

#include "node_watcher/Node.hpp"

#include "ai_msgs/NodeRequirement.h"
#include "ai_msgs/AwaitNodesRequest.h"
#include "ai_msgs/AwaitNodesResult.h"

#include <ros/ros.h>

#include <iostream>
#include <map>
#include <vector>

const int OPTIONAL = 1;
const int ALIVE = 2;

class NodesAwaiter
{
public:
	NodesAwaiter(
		ai_msgs::AwaitNodesRequest::Request& req,
		ai_msgs::AwaitNodesRequest::Response& res,
		ros::Publisher& resultPub
	);

	void updateStatus(std::string nodename, NodeStatus status);
	bool isFinished() const;

	static int lastRequestCode;
protected:
	ros::Publisher resultPub;
	ros::Timer timeout;
	ros::NodeHandle nh;

	int requestCode;
	bool finished;
	std::map<std::string, int> requirements;

	void checkFinished();
	void sendResults();
	void onTimeout(const ros::TimerEvent& timer);
};

bool operator <(const ai_msgs::NodeRequirement &lhs, const ai_msgs::NodeRequirement &rhs);

#endif