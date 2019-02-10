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

const std::string NODES_AWAITER_SERVICE = "/ai/node_watcher/wait"; 
const std::string NODES_AWAITER_RESULT_TOPIC = "/ai/node_watcher/wait_result"; 

const int OPTIONAL = 1;
const int ALIVE = 2;

class NodesAwaiter
{
public:
	NodesAwaiter(
		ai_msgs::AwaitNodesRequest::Request& req,
		ai_msgs::AwaitNodesRequest::Response& res,
		ros::Publisher& resultPub,
		ros::NodeHandle& nh
	);

	void updateStatus(std::string nodename, NodeStatus status);
	bool isFinished() const;

	static int lastRequestCode;
protected:
	ros::Publisher resultPub;
	ros::Timer timeout;

	int requestCode;
	bool finished;
	std::map<std::string, int> requirements;

	void checkFinished();
	void sendResults(bool timered = false);
	void onTimeout(const ros::TimerEvent& timer);
};

bool operator <(const ai_msgs::NodeRequirement &lhs, const ai_msgs::NodeRequirement &rhs);

#endif