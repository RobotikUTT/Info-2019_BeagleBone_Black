#include "node_watcher/NodeStatusHandler.hpp"

NodeStatusHandler::NodeStatusHandler()
    : nh() {
    
    // Init node_watcher service client
    watcherClient = nh.serviceClient<NodeReadiness>(Topics::NODE_WATCHER_SERVICE);
    
    // Wait for service to send init signal
    ros::service::waitForService(Topics::NODE_WATCHER_SERVICE);
}

string NodeStatusHandler::makeNodePath(string nodename, string package) {
    return "/" + package + "/" + nodename;
}

/**
 * Set given node status 
 */
void NodeStatusHandler::setNodeStatus(string nodename, string package, int state_code, int errorCode /*= 0*/) {
    this->setNodeStatus(this->makeNodePath(nodename, package), state_code, errorCode);
}

/**
 * Set given node status 
 */
void NodeStatusHandler::setNodeStatus(string nodepath, int state_code, int errorCode /*= 0*/) {
    // update remote version
    NodeReadiness msg;
    msg.request.status.state_code = state_code;
    msg.request.status.error_code = errorCode;
    msg.request.node_name = nodepath;

    if (watcherClient.call(msg)) {
        return;
    }

    ROS_ERROR_STREAM(
        "Unable to call watcher as setter for " << nodepath
    );
}

/**
 * Get an other node's status, with it's package name provided
 */
NodeStatus NodeStatusHandler::getNodeStatus(string nodename, string package) {
    return getNodeStatus(this->makeNodePath(nodename, package));
}

/**
 * Get an other node's status
 */
NodeStatus NodeStatusHandler::getNodeStatus(string nodepath) {
    NodeReadiness msg;
    msg.request.status.state_code = NodeStatus::NODE_ASKING; // ask for status
    msg.request.node_name = nodepath;

    if (watcherClient.call(msg)) {
        return msg.response.status;
    }

    ROS_ERROR_STREAM(
        "Unable to call watcher as getter for " << nodepath
    );
}