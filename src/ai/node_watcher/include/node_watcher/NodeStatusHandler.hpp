#ifndef ABSTRACT_NODE_HANDLER_HPP
#define ABSTRACT_NODE_HANDLER_HPP

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "ai_msgs/AwaitNodesRequest.h"
#include "ai_msgs/AwaitNodesResult.h"
#include "ai_msgs/NodeReadiness.h"
#include "ai_msgs/NodeStatus.h"
#include "ai_msgs/NodeRequirement.h"
#include "ai_msgs/Topics.h"

#include "std_msgs/Int32.h"

using std::string;
using ros::ServiceClient;

using namespace ai_msgs;

typedef boost::function<void(bool)> WaitCallback;

/**
 * Class handling any "abstract nodes" (as they are not defined as a single class)
 */
class NodeStatusHandler {
private:
    ServiceClient watcherClient;
    ServiceClient waiterClient;
    ros::Publisher startWaitPub;
    ros::Subscriber waitAnswerSub;
    
    WaitCallback waitCallback;

    std::vector<NodeRequirement> requirements;
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

    // Node waiting
    virtual void waitForNodes(int timeout);

    // Callback setter
    void setWaitCallback(WaitCallback cb);

    // Add node requirement
    void requireFromFile(std::string filename);
    void require(std::string nodepath, bool required = true);
    void require(std::string nodename, std::string package, bool required = true);

    // Requirement getter
    bool isRequired(string nodepath);
    
protected:
    ros::NodeHandle nh;

    virtual void onAwaitResponse(const AwaitNodesResult& msg);
    
    int waitRequestCode;
};

#endif