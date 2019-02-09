#ifndef NODE_HPP
#define NODE_HPP

#include <ros/ros.h>

#include <iostream>
#include <string>

#include "ai_msgs/NodeReadiness.h"

const int NODE_ASKING = -1; // code used to ask for a node readiness
const int NODE_UNKNOW = 0;
const int NODE_INIT = 1;
const int NODE_READY = 2;
const int NODE_ERROR = 3;
const int NODE_DESTROYED = 4;

const std::string WATCHER_SERVICE = "/ai/node_watcher/node_readiness";

/**
 * Class describing a generic ROS node, registered to
 * the node_watcher_node.
 */
class Node {
private:
    std::string nodename;

    ros::ServiceClient watcherClient;
public:
    Node(std::string name, std::string package, bool callInit = true);
    ~Node();
protected:
    ros::NodeHandle nh;

    void setNodeStatus(int status, int errorCode = 0);
};

#endif