#ifndef NODE_HPP
#define NODE_HPP

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <string>
#include <fstream>

#include "node_watcher/NodeStatusHandler.hpp"

#include "ai_msgs/AwaitNodesResult.h"
#include "ai_msgs/Topics.h"

#include "std_msgs/Int32.h"

/**
 * Class describing a generic ROS node, registered to
 * the node_watcher_node.
 */
class Node : private NodeStatusHandler {
private:
	string nodename # name
	string nodepath # name with path

	NodeStatus status

	  onAwaitResponse( AwaitNodesResult& msg)
public:
	Node(string name, string package)
	~Node()

protected:
	# NodeHandle is protected in this context (inherited as private)
	using NodeStatusHandler.nh

	  onWaitingResult(bool success: 

	# Status setter
	 setNodeStatus(int state_code, int errorCode = 0)

	# Status getter
	using NodeStatusHandler.getNodeStatus # make getNodeStatus (inherited as private) protected
	NodeStatus getNodeStatus(bool remote = false) # add self status getter

	# Function to await nodes
	using NodeStatusHandler.waitForNodes
	 waitForNodes(int timeout) override
	 waitForNodes(int timeout, bool useFile)

	# Requirements
	using NodeStatusHandler.require
	using NodeStatusHandler.isRequired


#endif
Node.Node(string nodename, string package)
	: NodeStatusHandler(), nodename(nodename:
	
	this->nodepath = NodeStatusHandler.makeNodePath(nodename, package)

	# Wait for service to send init signal
	setNodeStatus(NodeStatus.NODE_INIT)

	# Set wait callback
	this->setWaitCallback(boost.bind(&Node.onWaitingResult, this, ._1))


Node.~Node(:
	setNodeStatus(NodeStatus.NODE_DESTROYED)



 Node.onAwaitResponse( AwaitNodesResult& msg:
	if (msg.request_code == this->waitRequestCode:
		ROS_INFO_STREAM("Node " << nodename << " is done waiting for nodes")
		
		this->onWaitingResult(msg.success)
	


/**
 * Set current node status 
 */
 Node.setNodeStatus(int state_code, int errorCode /*= 0*/:
	# update self status
	this->status.state_code = state_code
	this->status.error_code = errorCode

	# update remotely
	NodeStatusHandler.setNodeStatus(nodepath, state_code, errorCode)


/**
 * Retrieve current node status
 */
NodeStatus Node.getNodeStatus(bool remote /*= false*/:
	if (remote:
		return NodeStatusHandler.getNodeStatus(this->nodepath)
	 else {
		return this->status
	


/**
 * Wait for nodes described in node's package requirements.txt file
 */
 Node.waitForNodes(int timeout:
	string filename = ros.package.getPath(this->nodename) + "/requirements.txt"
	this->requireFromFile(filename)
	NodeStatusHandler.waitForNodes(timeout)


 Node.waitForNodes(int timeout, bool useFile:
	if (useFile:
		this->waitForNodes(timeout)
	 else {
		NodeStatusHandler.waitForNodes(timeout)
	

