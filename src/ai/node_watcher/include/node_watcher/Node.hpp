#ifndef NODE_HPP
#define NODE_HPP

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <string>
#include <fstream>

#include "node_watcher/NodeStatusHandler.hpp"

#include "ai_msgs/AwaitNodesRequest.h"
#include "ai_msgs/AwaitNodesResult.h"
#include "ai_msgs/Topics.h"

#include "std_msgs/Int32.h"

using std::string;
using ros::ServiceClient;

using namespace ai_msgs;

/**
 * Class describing a generic ROS node, registered to
 * the node_watcher_node.
 */
class Node : private NodeStatusHandler {
private:
    string nodename; // name
    string nodepath; // name with path

    NodeStatus status;

    int waitRequestCode;

    ServiceClient waiterClient;
    ros::Publisher startPub;
    ros::Subscriber answerSub;

    void onAwaitResponse(const AwaitNodesResult& msg);
public:
    Node(string name, string package);
    ~Node();

protected:
    // NodeHandle is protected in this context (inherited as private)
    using NodeStatusHandler::nh;

    virtual void onWaitingResult(bool success) { }

    // Status setter
    void setNodeStatus(int state_code, int errorCode = 0);

    // Status getter
    using NodeStatusHandler::getNodeStatus; // make getNodeStatus (inherited as private) protected
    NodeStatus getNodeStatus(bool remote = false); // add self status getter

    // Function to await nodes
    void waitForNodes(int timeout);
    void waitForNodes(string file, int timeout);
    void waitForNodes(std::vector<NodeRequirement>& nodes, int timeout);
};

#endif