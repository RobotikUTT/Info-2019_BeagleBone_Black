#include "node_watcher/node_watcher_node.hpp"

NodeWatcher::NodeWatcher() : nh() {
    watcherService = nh.advertiseService(WATCHER_SERVICE, &NodeWatcher::nodeStatus, this);
}

bool NodeWatcher::nodeStatus(ai_msgs::NodeReadiness::Request& req, ai_msgs::NodeReadiness::Response& res) {
    int statusCode = req.status;
    
    // If we are not asking for the status (eg: use function as setter)
    if (statusCode != NODE_ASKING) {
        NodeStatus status;

        // Save data
        status.status = req.status;
        status.errorCode = req.error_code;

        nodes[req.node_name] = status;

        // Return same data
        res.status = req.status;
        res.error_code = req.error_code;
    } else {
        // Otherwise it is a getter
        auto found = nodes.find(req.node_name);

        // Found element with this name
        if (found != nodes.end()) {
            res.status = found->second.status;
            res.error_code = found->second.errorCode;
        } else { // otherwise
            res.status = NODE_UNKNOW;
            res.error_code = 0;
        }
    }

    return true;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "node_watcher_node");
	NodeWatcher node;
	ros::spin();
}
