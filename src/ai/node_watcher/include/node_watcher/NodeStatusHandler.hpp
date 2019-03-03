#ifndef ABSTRACT_NODE_HANDLER_HPP
#define ABSTRACT_NODE_HANDLER_HPP

#include <ros/ros.h>
#include <ros/package.h>

#include <string>

#include "ai_msgs/NodeReadiness.h"
#include "ai_msgs/NodeStatus.h"
#include "ai_msgs/Topics.h"

#include "std_msgs/Int32.h"

using std::string;
using ros::ServiceClient;

using namespace ai_msgs;

/**
 * Class handling any "abstract nodes" (as they are not defined as a single class)
 */
class NodeStatusHandler {
private:
    ServiceClient watcherClient;

public:
    NodeStatusHandler();

    // Build node path
    string makeNodePath(string nodename, string package);

    // Status getter
    NodeStatus getNodeStatus(string nodename, string package);
    NodeStatus getNodeStatus(string nodepath);
    
    // Status setter
    void setNodeStatus(string nodename, string package, int state_code, int errorCode = 0);
    void setNodeStatus(string nodepath, int state_code, int errorCode = 0);

protected:
    ros::NodeHandle nh;
};

#endif