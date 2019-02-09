#ifndef NODE_WATCHER_HPP
#define NODE_WATCHER_HPP

#include <ros/ros.h>

#include <iostream>
#include <map>

#include "node_watcher/Node.hpp"

#include "ai_msgs/NodeReadiness.h"

struct NodeStatus {
    int status;
    int errorCode;
};


class NodeWatcher
{
private:
    ros::NodeHandle nh;
    ros::ServiceServer watcherService;
    std::map<std::string, NodeStatus> nodes;
public:
    NodeWatcher();

    bool nodeStatus(ai_msgs::NodeReadiness::Request& req, ai_msgs::NodeReadiness::Response& res);
};




#endif