#include "node_watcher/NodesAwaiter.hpp"

NodesAwaiter::NodesAwaiter(ai_msgs::AwaitNodesRequest::Request& req, ai_msgs::AwaitNodesRequest::Response& res, ros::Publisher& pub, ros::NodeHandle& nh)
	: finished(false), resultPub(pub) {

	// Get a request code and set it as response
	this->requestCode = NodesAwaiter::lastRequestCode++;
	res.request_code = this->requestCode;

	// Extract data
	for (ai_msgs::NodeRequirement& node : req.nodes) {
		this->requirements[node.nodename] = node.optional ? OPTIONAL : 0;
	}

	// Set timeout callback (last true is for oneshot parameter)
	timeout = nh.createTimer(ros::Duration(req.timeout), &NodesAwaiter::onTimeout, this, true);

	this->checkFinished();
}

// Static request code counter 
int NodesAwaiter::lastRequestCode = 0;

void NodesAwaiter::updateStatus(std::string nodename, NodeStatus status) {
	// Do not update when finished
	if (this->finished) {
		return;
	}

	bool nodealive = (status.status == NODE_READY);

	// Search for element with given name
	auto found = this->requirements.find(nodename);
	if (found != this->requirements.end()) {
		found->second = (found->second & OPTIONAL) + nodealive * ALIVE;
		
		// If it's a true value, we check if it's finished
		if (nodealive) {
			this->checkFinished();
			return;
		}
	}


}

/**
 * Check if the wait is over and send finish message if all nodes
 * are alive
 */
void NodesAwaiter::checkFinished() {
	for (const auto& next : this->requirements) {
		ROS_INFO_STREAM("node " << next.first << " has state " << next.second);
		// If some node isn't alive
		if ((next.second & ALIVE) == 0) {
			ROS_INFO_STREAM("not alive leaving");
			return;
		}
	}

	ROS_INFO_STREAM("sending results");


	// If all nodes are alive
	this->sendResults();
	return;
}

void NodesAwaiter::onTimeout(const ros::TimerEvent& timer) {
	this->sendResults(true);
}

void NodesAwaiter::sendResults(bool timered /*= false*/) {
	// Make sure result is sent once and cancel timeout
	if (this->finished) return;
	this->finished = true;

	if (!timered) {
		timeout.stop();
	}

	ai_msgs::AwaitNodesResult result;
	result.success = true;
	result.request_code = this->requestCode;
	
	// Add missing nodes if any
	for (const auto& next : this->requirements) {
		ROS_INFO_STREAM("node " << next.first << " has state " << next.second);

		// If some node isn't alive
		if ((next.second & ALIVE) == 0) {
			// Rebuild requirement
			ai_msgs::NodeRequirement requirement;
			requirement.nodename = next.first; // duplicate string to avoid error
			requirement.optional = (next.second & OPTIONAL) > 0;

			// Push to missing ones
			result.missing_nodes.push_back(requirement);

			// If that node is mandatory
			if ((next.second & OPTIONAL) == 0) {
				// Set success to false
				result.success = false;
			}
		}
	}

	// Publish result
	this->resultPub.publish(result);

}

bool NodesAwaiter::isFinished() const {
	return this->finished;
}

bool operator <(const ai_msgs::NodeRequirement &lhs, const ai_msgs::NodeRequirement &rhs) {
	return lhs.nodename < rhs.nodename;
}