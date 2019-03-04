#include "node_watcher/Node.hpp"

Node::Node(string nodename, string package)
    : NodeStatusHandler(), nodename(nodename) {
    
    this->nodepath = NodeStatusHandler::makeNodePath(nodename, package);

    // Wait for service to send init signal
    setNodeStatus(NodeStatus::NODE_INIT);

    // Set wait callback
    this->setWaitCallback(boost::bind(&Node::onWaitingResult, this, ::_1));
}

Node::~Node() {
    setNodeStatus(NodeStatus::NODE_DESTROYED);
}


void Node::onAwaitResponse(const AwaitNodesResult& msg) {
    if (msg.request_code == this->waitRequestCode) {
        ROS_INFO_STREAM("Node " << nodename << " is done waiting for nodes");
        
        this->onWaitingResult(msg.success);
    }
}

/**
 * Set current node status 
 */
void Node::setNodeStatus(int state_code, int errorCode /*= 0*/) {
    // update self status
    this->status.state_code = state_code;
    this->status.error_code = errorCode;

    // update remotely
    NodeStatusHandler::setNodeStatus(nodepath, state_code, errorCode);
}

/**
 * Retrieve current node status
 */
NodeStatus Node::getNodeStatus(bool remote /*= false*/) {
    if (remote) {
        return NodeStatusHandler::getNodeStatus(this->nodepath);
    } else {
        return this->status;
    }
}

/**
 * Wait for nodes described in node's package requirements.txt file
 */
void Node::waitForNodes(int timeout) {
    string filename = ros::package::getPath(this->nodename) + "/requirements.txt";
    this->requireFromFile(filename);
    NodeStatusHandler::waitForNodes(timeout);
}

void Node::waitForNodes(int timeout, bool useFile) {
    if (useFile) {
        this->waitForNodes(timeout);
    } else {
        NodeStatusHandler::waitForNodes(timeout);
    }
}
