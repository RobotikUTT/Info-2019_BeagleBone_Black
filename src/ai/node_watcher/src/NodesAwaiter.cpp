#include "node_watcher/NodesAwaiter.hpp"

NodesAwaiter::NodesAwaiter(AwaitNodesRequest::Request& req, AwaitNodesRequest::Response& res, ros::Publisher& pub)
	: finished(false), sent(false), started(false), resultPub(pub), nh() {

	// Get a request code and set it as response
	this->requestCode = NodesAwaiter::lastRequestCode++;
	res.request_code = this->requestCode;

	// Extract data
	for (NodeRequirement& node : req.nodes) {
		this->requirements[node.nodename] = node.optional ? OPTIONAL : 0;
	}

	this->timeout = req.timeout;

	// Send result now if there is no action to wait
	if (req.nodes.size() == 0) {
		finished = true;
	}
}

// Static request code counter 
int NodesAwaiter::lastRequestCode = 0;

void NodesAwaiter::startRequested(int code) {
	if (code == this->requestCode) {
		started = true;

		if (finished) {
			// If finished send results right away
			sendResults();
		} else {
			// Set timeout callback (last true is for oneshot parameter)
			timer = nh.createTimer(ros::Duration(this->timeout), &NodesAwaiter::onTimeout, this, true);
		}
	}
}

void NodesAwaiter::updateStatus(std::string nodename, NodeStatus status) {
	// Do not update when finished
	if (this->finished) {
		return;
	}

	bool nodealive = (status.state_code == NodeStatus::NODE_READY);

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

	// If all nodes are alive, we send result
	finished = true;
	sendResults();
}

void NodesAwaiter::onTimeout(const ros::TimerEvent& timer) {
	this->sendResults();
}

void NodesAwaiter::sendResults() {
	// Make sure result is sent once and cancel timeout
	if (this->sent || !this->started) return;
	this->sent = true;

	timer.stop();

	AwaitNodesResult result;
	result.success = true;
	result.request_code = this->requestCode;
	
	// Add missing nodes if any
	for (const auto& next : this->requirements) {
		// If some node isn't alive
		if ((next.second & ALIVE) == 0) {
			// Rebuild requirement
			NodeRequirement requirement;
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

bool NodesAwaiter::isSent() const {
	return this->sent;
}

bool operator <(const NodeRequirement &lhs, const NodeRequirement &rhs) {
	return lhs.nodename < rhs.nodename;
}