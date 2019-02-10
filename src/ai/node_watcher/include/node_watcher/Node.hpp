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
 * Struct containing a node status
 */
struct NodeStatus {
    int status;
    int errorCode;
};


/**
 * Class describing a generic ROS node, registered to
 * the node_watcher_node.
 */
class Node {
private:
    std::string nodename;
    NodeStatus status;

    ros::ServiceClient watcherClient;
public:
    Node(std::string name, std::string package);
    ~Node();

protected:
    ros::NodeHandle nh;

    // Status setter
    void setNodeStatus(int status, int errorCode = 0);

    // Status getter
    NodeStatus getNodeStatus(bool remote = false);
    NodeStatus getNodeStatus(std::string nodename, std::string package);
    NodeStatus getNodeStatus(std::string nodename);
};

#endif