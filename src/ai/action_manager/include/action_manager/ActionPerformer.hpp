#ifndef ACTION_PERFORMER_H
#define ACTION_PERFORMER_H

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>

#include "ai_msgs/PerformAction.h"
#include "ai_msgs/Argument.h"
#include "ai_msgs/ComputeActionPoint.h"
#include "ai_msgs/RobotStatus.h"
#include "ai_msgs/ActionStatus.h"
#include "ai_msgs/ActionPoint.h"

#include "action_manager/Action.hpp"
#include "action_manager/Argumentable.hpp"

#include "node_watcher/Node.hpp"

#include "interface_msgs/StmDone.h"
#include "geometry_msgs/Pose2D.h"

typedef actionlib::SimpleActionServer<ai_msgs::PerformAction> PerformActionSrv;

using ai_msgs::RobotStatus;
using ai_msgs::ActionStatus;
using ai_msgs::Argument;

using std::string;

/**
 * Represent an performer for a specific action, it advertise a service for
 * predicting position and an action server to run the action.
 * 
 * It provide simple interface for creating robot action, and avoid tedious ROS objects
 * manipulation.
 */
class ActionPerformer : public Node, public Argumentable
{
public:
	ActionPerformer(std::string name);
	
protected:
	// Function defined by inherited actions
	virtual ActionPoint computeActionPoint(Argumentable* actionArgs, Pose2D robotPos) = 0;
	virtual void start() = 0;
	virtual void cancel() {};

	// Function managing the action
	void actionPerformed();
	void actionPaused();
private:
	std::vector<ai_msgs::Argument> _args;

	// Name of the perfomer
	std::string name;

	// Ros objects
	PerformActionSrv* actionServer;
	ros::ServiceServer actionPointSrv;
	ros::Subscriber robotWatcherSub;

	bool _computeActionPoint(
		ai_msgs::ComputeActionPoint::Request& req,
		ai_msgs::ComputeActionPoint::Response& res
	);

	void onGoal();
	void onPreempt();

	void onRobotStatus(const RobotStatus::ConstPtr& msg);
};

#endif
