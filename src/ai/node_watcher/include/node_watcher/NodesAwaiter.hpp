#ifndef NODES_AWAITER_HPP
#define NODES_AWAITER_HPP

#include <ros/ros.h>

#include "ai_msgs/NodeRequirement.h"
#include "ai_msgs/AwaitNodesRequest.h"
#include "ai_msgs/AwaitNodesResult.h"
#include "ai_msgs/NodeStatus.h"

#include "node_watcher/Node.hpp"

#include "std_msgs/Int32.h"

#include <iostream>
#include <map>
#include <vector>

using namespace ai_msgs;

const int OPTIONAL = 1;
const int ALIVE = 2;

class NodesAwaiter
{
public:
	NodesAwaiter(
		AwaitNodesRequest::Request& req,
		AwaitNodesRequest::Response& res,
		ros::Publisher& resultPub
	);

	void updateStatus(std::string nodename, NodeStatus status);
	bool isSent() const;
	void startRequested(int code);
	
	static int lastRequestCode;
protected:
	ros::Publisher resultPub;

	ros::Timer timer;
	ros::NodeHandle nh;

	int requestCode;
	int timeout;
	bool finished;
	bool started;
	bool sent;
	std::map<std::string, int> requirements;

	void checkFinished();
	void sendResults();
	void onTimeout(const ros::TimerEvent& timer);
};

bool operator <(const NodeRequirement &lhs, const NodeRequirement &rhs);

#endif