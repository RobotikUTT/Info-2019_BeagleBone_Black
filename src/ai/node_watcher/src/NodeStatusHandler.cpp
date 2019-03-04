#include "node_watcher/NodeStatusHandler.hpp"

NodeStatusHandler::NodeStatusHandler()
    : nh(), waitRequestCode(-1) {
    // Init node_watcher service client
    waiterClient = nh.serviceClient<AwaitNodesRequest>(Topics::NODES_AWAITER_INIT_SERVICE);
    watcherClient = nh.serviceClient<NodeReadiness>(Topics::NODE_WATCHER_SERVICE);
    
    // Send signal to start waiting
    startWaitPub = nh.advertise<std_msgs::Int32>(Topics::NODES_AWAITER_START_TOPIC, 10);
    
    // Listen for answers
    waitAnswerSub = nh.subscribe(Topics::NODES_AWAITER_RESULT_TOPIC, 10, &NodeStatusHandler::onAwaitResponse, this);

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

/**
 * Wait for required nodes to be alive
 */
void NodeStatusHandler::waitForNodes(int timeout) {
    if (requirements.size() == 0) {
        this->waitCallback(true);
    }

    // Prepare request
    AwaitNodesRequest request;
    request.request.nodes = requirements;
    request.request.timeout = timeout;
    
    // Call service to retrieve code
    if (waiterClient.call(request)) {
        this->waitRequestCode = request.response.request_code;

        // Wait for publisher to be ready to address not sent issue
        // From [https://answers.ros.org/question/11167/how-do-i-publish-exactly-one-message/]
        ros::Rate poll_rate(100);
        while(startWaitPub.getNumSubscribers() == 0) {
            poll_rate.sleep();
        }

        std_msgs::Int32 msg;
        msg.data = this->waitRequestCode;
        startWaitPub.publish(msg);
    } else {
        ROS_ERROR_STREAM("Unabled to call nodes waiting service");
        this->waitCallback(false);
    }
}

/**
 * Set the callback for waiting
 */
void NodeStatusHandler::setWaitCallback(WaitCallback cb) {
    this->waitCallback = cb;
}

void NodeStatusHandler::onAwaitResponse(const AwaitNodesResult& msg) {
    if (msg.request_code == this->waitRequestCode) {
        this->waitCallback(msg.success);
    }
}

/**
 * Add given node to required nodes
 */
void NodeStatusHandler::require(std::string nodename, std::string package, bool required /*= true*/) {
    this->require(this->makeNodePath(nodename, package), required);
}

/**
 * Add given node to required nodes
 */
void NodeStatusHandler::require(std::string nodepath, bool required /*= true*/) {
    NodeRequirement req;
    req.nodename = nodepath;
    req.optional = !required;

    this->requirements.push_back(req);
}

/**
 * Add nodes contained in given file to required nodes
 * 
 * The file should respect the following format (one node per line):
 * [nodename] required|optional
 */
void NodeStatusHandler::requireFromFile(std::string filename) {
    std::ifstream filestream;
    
    filestream.open(filename);

    if (filestream.fail()) {
        ROS_ERROR_STREAM("Unable to load dependency file : " + filename);
        this->waitCallback(false);
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
            // Check for error
            if (status != "optional" && status != "required") {
                ROS_WARN_STREAM("parsing requirements from " << filename << ": " << status
                    << " is not a valid optional argument (optional or required)");
            }

            // Save as required
            this->require(nodename, status == "required");
        } else {
            ROS_WARN_STREAM("parsing requirements from " << filename << ": line " << linenumber
                << " ignored due to lack of args");
        }

        linenumber += 1;
    }
}

bool NodeStatusHandler::isRequired(string nodepath) {
    for (const auto& nextReq : this->requirements) {
        if (nextReq.nodename == nodepath) {
            return true;
        }
    }

    return false;
}