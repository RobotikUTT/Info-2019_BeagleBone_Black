#include "node_watcher/Node.hpp"

Node::Node(std::string nodename, std::string package, bool callInit /*= true*/)
    : nh(), nodename("/" + package + "/" + nodename) {

    // Init node_watcher service client
    watcherClient = nh.serviceClient<ai_msgs::NodeReadiness>(WATCHER_SERVICE);

    // If an initialisation call is requested
    if (callInit && ros::service::waitForService(WATCHER_SERVICE)) {
        
        setNodeStatus(NODE_INIT);
    }

}

Node::~Node() {
    setNodeStatus(NODE_DESTROYED);
}

void Node::setNodeStatus(int status, int errorCode /*= 0*/) {
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