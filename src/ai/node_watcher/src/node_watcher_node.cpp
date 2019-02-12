#include "node_watcher/node_watcher_node.hpp"

NodeWatcher::NodeWatcher() : nh() {
    watcherService = nh.advertiseService(WATCHER_SERVICE, &NodeWatcher::nodeStatus, this);
    waiterService = nh.advertiseService(NODES_AWAITER_SERVICE, &NodeWatcher::awaitNodes, this);

    updatePublisher = nh.advertise<NodeStatusUpdate>("/ai/node_watcher/update", 100);
    waitResultPublisher = nh.advertise<AwaitNodesResult>(NODES_AWAITER_RESULT_TOPIC, 100);
}

NodeStatus NodeWatcher::getNodeStatus(std::string name) {
    auto found = nodes.find(name);

    // Found element with this name
    if (found != nodes.end()) {
        return found->second;
    } else { // otherwise
        NodeStatus unknowStatus;
        unknowStatus.state_code = NodeStatus::NODE_UNKNOW;
        
        return unknowStatus;
    }
}

bool NodeWatcher::awaitNodes(AwaitNodesRequest::Request& req, AwaitNodesRequest::Response& res) {
    std::shared_ptr<NodesAwaiter> waiter = std::make_shared<NodesAwaiter>(req, res, waitResultPublisher);

    // Add all current states to node
    for (const auto& next : this->nodes) {
		waiter->updateStatus(next.first, next.second);
	}
    
    // Add to list of waiters if not already done
    if (!waiter->isFinished()) {
        this->waiters.push_back(waiter);
    }

    return true;
}
/**
 * Change a node status for all waiters, and clear done waiters as well
 */
void NodeWatcher::updateWaiters(std::string name, NodeStatus status) {
    std::vector<std::shared_ptr<NodesAwaiter>>::iterator i = waiters.begin();

    // Loop from [https://stackoverflow.com/questions/596162/can-you-remove-elements-from-a-stdlist-while-iterating-through-it]
    while (i != waiters.end()) {
        // Remove done elements
        if ((*i)->isFinished()) {
            i = waiters.erase(i);
        }
        
        // Otherwise update them
        else {
            (*i)->updateStatus(name, status);
            ++i;
        }
    }
}

bool NodeWatcher::nodeStatus(NodeReadiness::Request& req, NodeReadiness::Response& res) {
    NodeStatus status = req.status;
    
    // If we are not asking for the status (eg: use function as setter)
    if (status.state_code != NodeStatus::NODE_ASKING) {
        nodes[req.node_name] = status;

        // Return same data
        res.status = status;

        // Publish update to subscribers
        NodeStatusUpdate updateMsg;
        updateMsg.status = status;
        updateMsg.node_name = req.node_name;

        updatePublisher.publish(updateMsg);

        this->updateWaiters(req.node_name, status);

        ROS_INFO_STREAM("Node " << req.node_name << " registered with status " << STATUS_STRINGS[req.status.state_code]);
    } else {
        // Otherwise it is a getter
        res.status = getNodeStatus(req.node_name);
    }

    return true;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "node_watcher_node");
	NodeWatcher node;
	ros::spin();
}
