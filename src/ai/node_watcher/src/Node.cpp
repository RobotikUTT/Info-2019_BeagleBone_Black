#include "node_watcher/Node.hpp"

Node::Node(string nodename, string package)
    : NodeStatusHandler(), nodename(nodename), waitRequestCode(-1) {
    
    this->nodepath = NodeStatusHandler::makeNodePath(nodename, package);

    // Send signal to start waiting
    startPub = nh.advertise<std_msgs::Int32>(Topics::NODES_AWAITER_START_TOPIC, 10);
    
    // Init node_watcher service client
    waiterClient = nh.serviceClient<AwaitNodesRequest>(Topics::NODES_AWAITER_INIT_SERVICE);
    
    // Listen for answers
    answerSub = nh.subscribe(Topics::NODES_AWAITER_RESULT_TOPIC, 10, &Node::onAwaitResponse, this);

    // Wait for service to send init signal
    setNodeStatus(NodeStatus::NODE_INIT);
}

Node::~Node() {
    setNodeStatus(NodeStatus::NODE_DESTROYED);
}

/**
 * Wait for nodes described in node's package requirements.txt file
 * 
 * @return whether required nodes have proven to be alive
 */
void Node::waitForNodes(int timeout) {
    string filename = ros::package::getPath(this->nodename) + "/requirements.txt";

    waitForNodes(filename, timeout);
}

/**
 * Wait for nodes described in given file
 * 
 * @return whether required nodes have proven to be alive
 */
void Node::waitForNodes(string file, int timeout) {
    std::vector<NodeRequirement> nodes;
    std::ifstream filestream;
    
    filestream.open(file);

    if (filestream.fail()) {
        ROS_ERROR_STREAM("Unable to load dependency file : " + file);
        onWaitingResult(false);
        return;
    }

    // Parse file
    string nodename;
    string status;
    int linenumber = 1;

    // First element of line parsed as nodename
    while(std::getline(filestream, nodename, ' ')) {
        // Then the node optional argument
        if (std::getline(filestream, status)) {
            // Create requirement object
            NodeRequirement currentNode;
            currentNode.nodename = nodename;
            currentNode.optional = status == "optional";

            // Check for error
            if (!currentNode.optional && status != "required") {
                ROS_WARN_STREAM("parsing requirements from " << file << ": " << status
                    << " is not a valid optional argument (optional or required)");
            }

            // Push it to the list
            nodes.push_back(currentNode);
        } else {
            ROS_WARN_STREAM("parsing requirements from " << file << ": line " << linenumber
                << " ignored due to lack of args");
        }

        linenumber += 1;
    }

    waitForNodes(nodes, timeout);
}

/**
 * Wait for given nodes to be alive
 * 
 * @return whether required nodes have proven to be alive
 */
void Node::waitForNodes(std::vector<NodeRequirement>& nodes, int timeout) {
    ROS_INFO_STREAM("Node " << nodename << " is waiting for " << nodes.size() << " other nodes to start");
    
    if (nodes.size() == 0) {
        ROS_INFO_STREAM("Node " << nodename << " is done waiting for nodes");
        this->onWaitingResult(true);
    }

    // Prepare request
    AwaitNodesRequest request;
    request.request.nodes = nodes;
    request.request.timeout = timeout;
    
    // Call service to retrieve code
    if (waiterClient.call(request)) {
        this->waitRequestCode = request.response.request_code;

        // Wait for publisher to be ready to address not sent issue
        // From [https://answers.ros.org/question/11167/how-do-i-publish-exactly-one-message/]
        ros::Rate poll_rate(100);
        while(startPub.getNumSubscribers() == 0) {
            poll_rate.sleep();
        }

        std_msgs::Int32 msg;
        msg.data = this->waitRequestCode;
        startPub.publish(msg);
    } else {
        ROS_ERROR_STREAM("Unabled to call nodes waiting service");
        this->onWaitingResult(false);
    }
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
