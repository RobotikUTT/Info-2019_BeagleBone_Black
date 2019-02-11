#include "node_watcher/NodesAwaiter.hpp"

NodesAwaiter::NodesAwaiter(ai_msgs::AwaitNodesRequest::Request& req, ai_msgs::AwaitNodesRequest::Response& res, ros::Publisher& pub)
	: finished(false), resultPub(pub), nh() {

	// Get a request code and set it as response
	this->requestCode = NodesAwaiter::lastRequestCode++;
	res.request_code = this->requestCode;

	// Extract data
	for (ai_msgs::NodeRequirement& node : req.nodes) {
		this->requirements[node.nodename] = node.optional ? OPTIONAL : 0;
	}

	// Set timeout callback (last true is for oneshot parameter)
	timeout = nh.createTimer(ros::Duration(req.timeout), &NodesAwaiter::onTimeout, this, true);

	// Send result now	
	if (req.nodes.size() == 0) {
		this->sendResults();
	}
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
		// If some node isn't alive
		if ((next.second & ALIVE) == 0) {
			return;
		}
	}


	// If all nodes are alive, we wait just enough for service to return request_code
	// in case it was called before return
	nh.createTimer(ros::Duration(0.1), &NodesAwaiter::onTimeout, this, true);
	return;
}

void NodesAwaiter::onTimeout(const ros::TimerEvent& timer) {
	this->sendResults();
}

void NodesAwaiter::sendResults() {
	// Make sure result is sent once and cancel timeout
	if (this->finished) return;
	this->finished = true;

	timeout.stop();

	ai_msgs::AwaitNodesResult result;
	result.success = true;
	result.request_code = this->requestCode;
	
	// Add missing nodes if any
	for (const auto& next : this->requirements) {
		// If some node isn't alive
		if ((next.second & ALIVE) == 0) {
			// Rebuild requirement
			ai_msgs::NodeRequirement requirement;
			requirement.nodename = next.first;
			requirement.optional = (next.second & OPTIONAL) > 0;

			// Push to missing ones
			result.missing_nodes.push_back(requirement);

			
			// If that node is mandatory
			if ((next.second & OPTIONAL) == 0) {
				ROS_ERROR_STREAM("Some node required " << next.first << " but it did not woke up in time");

				// Set success to false
				result.success = false;
			} else {
				ROS_WARN_STREAM("Some node required " << next.first << " but it did not woke up in time");
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