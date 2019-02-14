#ifndef NODE_HPP
#define NODE_HPP

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <string>
#include <fstream>

#include <mutex>
#include <condition_variable>

#include "ai_msgs/NodeReadiness.h"
#include "ai_msgs/AwaitNodesRequest.h"
#include "ai_msgs/AwaitNodesResult.h"
#include "ai_msgs/NodeStatus.h"

#include "std_msgs/Int32.h"

using std::string;
using ros::ServiceClient;

using namespace ai_msgs;

const string STATUS_STRINGS[] = { "UNKNOW", "INIT", "READY", "ERROR", "DESTROYED" };

const string WATCHER_SERVICE = "/ai/node_watcher/node_readiness";

const string NODES_AWAITER_INIT_SERVICE = "/ai/node_watcher/wait"; 
const string NODES_AWAITER_START_TOPIC = "/ai/node_watcher/wait_start"; 
const string NODES_AWAITER_RESULT_TOPIC = "/ai/node_watcher/wait_result"; 

/**
 * Class describing a generic ROS node, registered to
 * the node_watcher_node.
 */
class Node {
private:
    string nodename; // name
    string nodepath; // name with path

    NodeStatus status;

    int waitRequestCode;

    ServiceClient watcherClient;
    ServiceClient waiterClient;
    ros::Publisher startPub;
    ros::Subscriber answerSub;

    void onAwaitResponse(const AwaitNodesResult& msg);
public:
    Node(string name, string package);
    ~Node();

protected:
    ros::NodeHandle nh;

    virtual void onWaitingResult(bool success) { }

    // Status setter
    void setNodeStatus(int state_code, int errorCode = 0);

    // Status getter
    NodeStatus getNodeStatus(bool remote = false);
    NodeStatus getNodeStatus(string nodename, string package);
    NodeStatus getNodeStatus(string nodename);

    // Function to await nodes
    void waitForNodes(int timeout);
    void waitForNodes(string file, int timeout);
    void waitForNodes(std::vector<NodeRequirement>& nodes, int timeout);
};

#endif