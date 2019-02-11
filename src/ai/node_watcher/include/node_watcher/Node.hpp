#ifndef NODE_HPP
#define NODE_HPP

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <string>
#include <fstream>

#include "ai_msgs/NodeReadiness.h"
#include "ai_msgs/AwaitNodesRequest.h"
#include "ai_msgs/AwaitNodesResult.h"

using std::string;
using ros::ServiceClient;

using ai_msgs::AwaitNodesResult;
using ai_msgs::AwaitNodesRequest;
using ai_msgs::NodeRequirement;
using ai_msgs::NodeReadiness;

const int NODE_ASKING = -1; // code used to ask for a node readiness
const int NODE_UNKNOW = 0;
const int NODE_INIT = 1;
const int NODE_READY = 2;
const int NODE_ERROR = 3;
const int NODE_DESTROYED = 4;

const string STATUS_STRINGS[] = { "UNKNOW", "INIT", "READY", "ERROR", "DESTROYED" };

const string WATCHER_SERVICE = "/ai/node_watcher/node_readiness";

const string NODES_AWAITER_SERVICE = "/ai/node_watcher/wait"; 
const string NODES_AWAITER_RESULT_TOPIC = "/ai/node_watcher/wait_result"; 

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
    string nodename; // name
    string nodepath; // name with path

    NodeStatus status;

    ServiceClient watcherClient;
    ServiceClient waiterClient;
public:
    Node(string name, string package);
    ~Node();

protected:
    ros::NodeHandle nh;

    // Status setter
    void setNodeStatus(int status, int errorCode = 0);

    // Status getter
    NodeStatus getNodeStatus(bool remote = false);
    NodeStatus getNodeStatus(string nodename, string package);
    NodeStatus getNodeStatus(string nodename);

    // Function to await nodes
    bool waitForNodes(int timeout);
    bool waitForNodes(string file, int timeout);
    bool waitForNodes(std::vector<NodeRequirement>& nodes, int timeout);
};

#endif