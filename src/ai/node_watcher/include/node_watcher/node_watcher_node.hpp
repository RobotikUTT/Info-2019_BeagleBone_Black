#ifndef NODE_WATCHER_HPP
#define NODE_WATCHER_HPP

#include <ros/ros.h>

#include <iostream>
#include <map>
#include <vector>
#include <memory>

#include "node_watcher/Node.hpp"
#include "node_watcher/NodesAwaiter.hpp"

#include "ai_msgs/NodeReadiness.h"
#include "ai_msgs/NodeStatusUpdate.h"

#include "ai_msgs/NodeRequirement.h"
#include "ai_msgs/AwaitNodesRequest.h"
#include "ai_msgs/AwaitNodesResult.h"

using namespace ai_msgs;

class NodeWatcher
{
private:
    ros::NodeHandle nh;
    ros::ServiceServer watcherService;
    ros::ServiceServer waiterService;

    ros::Publisher updatePublisher;
    ros::Publisher waitResultPublisher;

    std::map<std::string, NodeStatus> nodes;

    std::vector<std::shared_ptr<NodesAwaiter>> waiters;
public:
    NodeWatcher();

    bool nodeStatus(NodeReadiness::Request& req, NodeReadiness::Response& res);
    bool awaitNodes(AwaitNodesRequest::Request& req, AwaitNodesRequest::Response& res);
    
    void updateWaiters(std::string name, NodeStatus status);
    NodeStatus getNodeStatus(std::string name);
};




#endif