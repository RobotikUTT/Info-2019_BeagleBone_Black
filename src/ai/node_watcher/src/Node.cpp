#include "node_watcher/Node.hpp"

Node::Node(std::string nodename, std::string package)
    : nh(), nodename("/" + package + "/" + nodename) {

    // Init node_watcher service client
    watcherClient = nh.serviceClient<ai_msgs::NodeReadiness>(WATCHER_SERVICE);

    // Wait for service to send init signal
    if (ros::service::waitForService(WATCHER_SERVICE)) {
        setNodeStatus(NODE_INIT);
    }

}

Node::~Node() {
    setNodeStatus(NODE_DESTROYED);
}

/**
 * Set current node status 
 */
void Node::setNodeStatus(int status, int errorCode /*= 0*/) {
    // update self status
    this->status.status = status;
    this->status.errorCode = errorCode;

    // update remote version
    ai_msgs::NodeReadiness msg;
    msg.request.status = status;
    msg.request.node_name = nodename;
    msg.request.error_code = errorCode;

    if (watcherClient.call(msg)) {
        return;
    }

    ROS_ERROR_STREAM(
        "unable to call watcher for " << nodename
    );
}

/**
 * Retrieve current node status
 */
NodeStatus Node::getNodeStatus(bool remote /*= false*/) {
    if (remote) {
        return getNodeStatus(this->nodename);
    } else {
        return this->status;
    }
}

/**
 * Get an other node's status, with it's package name provided
 */
NodeStatus Node::getNodeStatus(std::string nodename, std::string package) {
    return getNodeStatus("/" + package + "/" + nodename);
}

/**
 * Get an other node's status
 */
NodeStatus Node::getNodeStatus(std::string nodename) {
    ai_msgs::NodeReadiness msg;
    msg.request.status = NODE_ASKING; // ask for status
    msg.request.node_name = nodename;

    if (watcherClient.call(msg)) {
        NodeStatus response;
        response.status = msg.response.status;
        response.errorCode = msg.response.error_code;
        return response;
    }

    ROS_ERROR_STREAM(
        "unable to call watcher for " << nodename
    );
}